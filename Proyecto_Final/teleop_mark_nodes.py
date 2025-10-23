# teleop_mark_nodes.py
# Teleoperación con flechas, marcado de nodos (G), pose (P), UND OCK (U), freno (Espacio).
# Segmentación robusta + telemetría y seguridad.
#
# Teclas:
#   ↑/←/↓/→  mover
#   G       guardar nodo
#   P       imprimir pose
#   U       undock (retrocede + gira según parámetros pasados)
#   C       clear safety (override inmediato)
#   ESPACIO freno emergencia (safety.brake)
#   ESC     salir

import queue, threading, time, math, sys, asyncio

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3
from pynput import keyboard
try:
    import tkinter as tk
    from tkinter import simpledialog
except Exception:
    tk = None
    simpledialog = None

import nodes_io
from core.undock import perform_undock
from core.safety import SafetyMonitorV2
from core.telemetry import TelemetryLogger
from core.config_validator import get_validated_config, print_config_summary

# --- Cargar configuración validada ---
try:
    config = get_validated_config()
    print_config_summary(config)
except ValueError as e:
    print(f"❌ Error de configuración: {e}")
    sys.exit(1)

# --- Conexión ---
robot = Create3(Bluetooth(config['robot']['name']))

# --- Parámetros desde config (valores probados + escalas de calibración) ---
VEL  = float(config['motion']['vel_default_cm_s'])
GIRO = float(config['motion']['giro_default_cm_s'])
ANCHO_EJE_CM = float(config['motion']['track_width_cm'])
LINEAR_SCALE  = float(config['motion'].get('linear_scale', 1.0))
ANGULAR_SCALE = float(config['motion'].get('angular_scale', 1.0))

# Aplicar escalas a los comandos efectivos
VEL_EFF  = VEL * LINEAR_SCALE
GIRO_EFF = GIRO * ANGULAR_SCALE

# Grados por segundo efectivos (para estimación planificada)
DEG_POR_SEG  = (2.0 * GIRO_EFF / ANCHO_EJE_CM) * (180.0 / math.pi)

# --- Estado teclas y colas ---
keys = {'up': False, 'left': False, 'down': False, 'right': False}
speed_q   = queue.Queue()     # (vl, vr)
control_q = queue.Queue()     # {"type":..., ...}
connected = threading.Event()
listener  = None
naming_mode = threading.Event()  # true mientras se pide nombre

# Velocidad inicial en 0
speed_q.put((0, 0))

# --- Estado de segmentación ---
state_start_t = time.perf_counter()
prev_state = "stop"
pending_segments = []  # acumulados desde el último nodo
last_node_id = None    # id del último nodo guardado

# Odómetro
latest_pose = None
seg_odom_start = None

# Seguridad y telemetría (async)
_safety: SafetyMonitorV2 = None
_telemetry: TelemetryLogger = None

def current_state_name() -> str:
    fwd = (1 if keys['up'] else 0) - (1 if keys['down'] else 0)
    ang = (-1 if keys['left'] else 0) + (1 if keys['right'] else 0)
    if fwd == 0 and ang == 0: return "stop"
    if fwd != 0 and ang == 0: return "forward" if fwd > 0 else "backward"
    if fwd == 0 and ang != 0: return "turn_right" if ang > 0 else "turn_left"
    if fwd > 0 and ang > 0:  return "curve_right"
    if fwd > 0 and ang < 0:  return "curve_left"
    if fwd < 0 and ang > 0:  return "curve_back_right"
    if fwd < 0 and ang < 0:  return "curve_back_left"
    return "moving"

def update_motion_queue():
    # Lineal efectiva (aplica LINEAR_SCALE)
    vl = VEL_EFF * ((1 if keys['up'] else 0) - (1 if keys['down'] else 0))
    vr = vl
    # Diferencial de giro efectivo (aplica ANGULAR_SCALE)
    if keys['left']:
        vl -= GIRO_EFF; vr += GIRO_EFF
    if keys['right']:
        vl += GIRO_EFF; vr -= GIRO_EFF
    while not speed_q.empty():
        try: speed_q.get_nowait()
        except: break
    speed_q.put((vl, vr))

def _normalize_angle_deg(a):
    while a >= 180.0: a -= 360.0
    while a <  -180.0: a += 360.0
    return a

def queue_close_segment():
    control_q.put({
        "type": "CLOSE_SEG",
        "t": time.perf_counter(),
        "state": prev_state
    })

def on_press(k):
    if not connected.is_set(): return
    # Teclas de acción (mayúsculas y minúsculas)
    try:
        if k == keyboard.Key.space:
            control_q.put({"type":"BRAKE"})
            return
        # Convertir a char y comparar en minúscula
        if hasattr(k, 'char') and k.char:
            char = k.char.lower()
            if char == 'u':
                control_q.put({"type":"UNDOCK"})
                return
            if char == 'g':
                control_q.put({"type":"REQ_NAME"})
                return
            if char == 'p':
                control_q.put({"type":"PRINT_POSE"})
                return
            if char == 'c':
                control_q.put({"type":"CLEAR_SAFETY"})
                return
    except: pass

    try:
        if naming_mode.is_set():
            return
        key_name = None
        if k == keyboard.Key.up: key_name = 'up'
        elif k == keyboard.Key.down: key_name = 'down'
        elif k == keyboard.Key.left: key_name = 'left'
        elif k == keyboard.Key.right: key_name = 'right'
        if key_name and not keys[key_name]:
            queue_close_segment()
            keys[key_name] = True
            global prev_state
            prev_state = current_state_name()
            update_motion_queue()
    except: pass

def on_release(k):
    if not connected.is_set(): return
    if k == keyboard.Key.esc:
        return False
    try:
        if naming_mode.is_set():
            return
        key_name = None
        if k == keyboard.Key.up: key_name = 'up'
        elif k == keyboard.Key.down: key_name = 'down'
        elif k == keyboard.Key.left: key_name = 'left'
        elif k == keyboard.Key.right: key_name = 'right'
        if key_name and keys[key_name]:
            queue_close_segment()
            keys[key_name] = False
            global prev_state
            prev_state = current_state_name()
            update_motion_queue()
    except: pass

async def _start_services():
    global _safety, _telemetry
    # Safety v2: no intrusivo, por defecto DESACTIVADO en teleop
    _safety = SafetyMonitorV2(robot, 
                              ir_threshold=config['safety']['ir_threshold'],
                              period_s=config['safety']['safety_period_s'])
    _safety.enable(config['safety']['enable_auto_brake'])
    await _safety.start()
    
    _telemetry = TelemetryLogger(robot, 
                                out_dir=config['telemetry']['log_dir'],
                                period_s=config['telemetry']['period_s'])
    await _telemetry.start()

@event(robot.when_play)
async def loop(robot):
    global last_node_id, prev_state, state_start_t, pending_segments, latest_pose, seg_odom_start, listener
    if not connected.is_set():
        connected.set()
        print("Conectado. Origen = Dock (reset_navigation).")
        await robot.reset_navigation()
        await robot.set_lights_on_rgb(255,255,255)

        # Servicios
        await _start_services()

        # inicializar odómetro
        p = await robot.get_position()
        try:
            latest_pose = type('Pose', (), dict(x=p.x, y=p.y, heading=p.heading))
        except:
            x,y,th = p
            latest_pose = type('Pose', (), dict(x=x, y=y, heading=th))
        seg_odom_start = latest_pose

        # Asegurar que robot está detenido
        await robot.set_wheel_speeds(0, 0)
        
        # Crear nodo Dock si no existe
        nodes = nodes_io.load_nodes()
        if not nodes:
            pose = await robot.get_position()
            try:
                x, y, th = pose.x, pose.y, pose.heading
            except AttributeError:
                x, y, th = pose
            dock = nodes_io.append_node(x,y,th,name="Dock", source="init")
            last_node_id = dock["id"]
            print(f"[Init] Nodo Dock creado id={last_node_id}")
        else:
            last_node_id = nodes[-1]["id"]

    while True:
        # aplicar velocidades (respetando safety si está habilitado)
        if not speed_q.empty():
            vl, vr = speed_q.get_nowait()
            if _safety and _safety.halted.is_set():
                # Safety activo - no aplicar velocidades
                await robot.set_wheel_speeds(0, 0)
                if _telemetry: _telemetry.update_command(0, 0)
            else:
                await robot.set_wheel_speeds(vl, vr)
                if _telemetry: _telemetry.update_command(vl, vr)

        # control
        if not control_q.empty():
            msg = control_q.get_nowait()

            if msg["type"] == "BRAKE":
                # Freno manual
                if _safety: await _safety.brake()
                await robot.set_wheel_speeds(0, 0)
                if _telemetry: _telemetry.update_command(0, 0)
                # reinicio de segmentación
                state_start_t = time.perf_counter()
                seg_odom_start = latest_pose
                prev_state = "stop"

            elif msg["type"] == "UNDOCK":
                # detener por seguridad
                await robot.set_wheel_speeds(0,0)
                if _telemetry: _telemetry.update_command(0,0)
                # ejecutar rutina estándar (parámetros desde config.yaml)
                undock_cfg = config['undock']
                await perform_undock(robot, 
                                   back_cm=undock_cfg['back_cm'],
                                   turn_deg=undock_cfg['turn_deg'], 
                                   turn_dir=undock_cfg['turn_dir'],
                                   back_speed=undock_cfg['back_speed'])
                # marcar nodo de inicio desde Dock
                pose = await robot.get_position()
                try: x,y,th = pose.x, pose.y, pose.heading
                except AttributeError: x,y,th = pose
                startn = nodes_io.append_node(x,y,th,name="StartFromDock", source="undock")
                last_node_id = startn["id"]
                print(f"[Undock] Nodo '{startn['name']}' id={last_node_id} @ [{x:.1f},{y:.1f},{th:.1f}]")

            elif msg["type"] == "CLOSE_SEG":
                close_t = msg.get("t", time.perf_counter())
                state_for_seg = msg.get("state", prev_state)
                dt = max(0.0, close_t - state_start_t)
                if dt >= 1e-3 and state_for_seg != "stop":
                    lin_prev = 0.0; ang_prev = 0.0
                    if ("forward" in state_for_seg) or ("curve" in state_for_seg) or (state_for_seg == "moving"):
                        # Usar velocidad lineal efectiva para la estimación planificada
                        if "back" in state_for_seg: lin_prev = -VEL_EFF
                        else: lin_prev = VEL_EFF
                    if ("turn_left" in state_for_seg) or ("curve_left" in state_for_seg) or ("curve_back_left" in state_for_seg):
                        ang_prev = -DEG_POR_SEG
                    if ("turn_right" in state_for_seg) or ("curve_right" in state_for_seg) or ("curve_back_right" in state_for_seg):
                        ang_prev = +DEG_POR_SEG
                    if state_for_seg in ("forward","backward"): ang_prev = 0.0
                    if state_for_seg in ("turn_left","turn_right"): lin_prev = 0.0

                    planned_dist = lin_prev * dt
                    planned_deg  = ang_prev * dt

                    if seg_odom_start is None:
                        seg_odom_start = latest_pose
                    dx = (latest_pose.x - seg_odom_start.x) if latest_pose else 0.0
                    dy = (latest_pose.y - seg_odom_start.y) if latest_pose else 0.0
                    real_dist = math.hypot(dx, dy)
                    real_deg  = 0.0
                    if latest_pose:
                        real_deg = _normalize_angle_deg(latest_pose.heading - seg_odom_start.heading)

                    seg = {
                        "state": state_for_seg,
                        "t": round(dt, 3),
                        "dist_cm": round(planned_dist, 2),
                        "deg": round(planned_deg, 2),
                        "odom_dist_cm": round(real_dist, 2),
                        "odom_deg": round(real_deg, 2),
                        "err_dist_cm": round(real_dist - planned_dist, 2),
                        "err_deg": round(_normalize_angle_deg(real_deg - planned_deg), 2)
                    }
                    pending_segments.append(seg)

                state_start_t = close_t
                seg_odom_start = latest_pose

            elif msg["type"] == "REQ_NAME":
                naming_mode.set()
                # Pausar listener para permitir input en terminal si no hay TK
                name = None
                if tk is not None and simpledialog is not None:
                    try:
                        root = tk.Tk(); root.withdraw()
                        name = simpledialog.askstring("Nuevo nodo", "Nombre del nodo:")
                        try: root.destroy()
                        except: pass
                    except Exception:
                        name = None
                if not name:
                    global listener
                    if listener and listener.running:
                        listener.stop(); listener = None
                    print("Nombre del nodo? (ENTER para 'Nodo'): ", end="", flush=True)
                    try:
                        name = input().strip()
                    except Exception:
                        name = "Nodo"
                if not name: name = "Nodo"

                # Cerrar segmento actual
                control_q.put({"type":"CLOSE_SEG", "t": time.perf_counter(), "state": prev_state})

                # Crear nodo y edge
                pose = await robot.get_position()
                try: x, y, th = pose.x, pose.y, pose.heading
                except AttributeError: x, y, th = pose
                node = nodes_io.append_node(x,y,th,name=name)
                print(f"\nGuardado nodo id={node['id']} '{node['name']}' [x={x:.1f}, y={y:.1f}, θ={th:.1f}]")
                if last_node_id is not None and last_node_id != node["id"]:
                    # agrega agregados/calidad simple
                    agg = nodes_io.aggregate_edge(pending_segments)
                    edge = nodes_io.append_edge(last_node_id, node["id"], pending_segments, agg=agg, quality=agg.get("quality"))
                    try:
                        nodes_io.log_edge_segments_csv(last_node_id, node["id"], pending_segments)
                    except Exception as e:
                        print(f"(log CSV falló: {e})")
                    print(f"Ruta registrada: {last_node_id} -> {node['id']} con {len(pending_segments)} segmento(s). Calidad={edge.get('quality')}")
                pending_segments = []
                last_node_id = node["id"]
                state_start_t = time.perf_counter()
                prev_state = "stop"
                try:
                    await robot.play_note(587, 0.12)
                except Exception:
                    pass
                await robot.set_lights_on_rgb(0,255,0)
                naming_mode.clear()

            elif msg["type"] == "PRINT_POSE":
                pose = await robot.get_position()
                try:
                    print(f"Pose: [x={pose.x:.1f}, y={pose.y:.1f}, θ={pose.heading:.1f}]")
                except AttributeError:
                    x, y, th = pose
                    print(f"Pose: [x={x:.1f}, y={y:.1f}, θ={th:.1f}]")
            
            elif msg["type"] == "CLEAR_SAFETY":
                if _safety:
                    await _safety.clear_halt()
                    print("✓ Safety: override aplicado")

        # actualizar pose reciente
        try:
            p = await robot.get_position()
            try:
                latest_pose = type('Pose', (), dict(x=p.x, y=p.y, heading=p.heading))
            except:
                x,y,th = p
                latest_pose = type('Pose', (), dict(x=x, y=y, heading=th))
        except Exception:
            pass

        await robot.wait(0.05)

if __name__ == "__main__":
    print("Teleop: ↑/←/↓/→ | G=guardar nodo | P=pose | U=undock | ESPACIO=freno | ESC=salir")
    t_robot = threading.Thread(target=robot.play, daemon=True)
    t_robot.start()
    if not connected.wait(timeout=20):
        print("No se pudo conectar."); sys.exit(1)

    # Listener de teclado
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    listener.join()  # Bloquea hasta ESC
