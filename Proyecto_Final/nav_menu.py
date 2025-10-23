# nav_menu.py
# GUI de navegación con tkinter para Create3
# Sistema principal de navegación con interfaz visual
# Cambios clave:
#  - "Ir a Origen" usa IRAvoidNavigator en lugar de navigate_to(0,0)
#  - Fuerza Safety habilitado durante navegación autónoma
#  - Limpieza/cancelación consistente de tareas antes de iniciar nueva navegación

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import threading, queue, sys, math, asyncio
import os
from typing import Dict, List, Optional

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

from nodes_io import (
    load_nodes, load_edges, nodes_index_by_id, nodes_index_by_name,
    resolve_node, neighbors_of, compute_missing_routes, log_nav_attempt
)
from core.undock import perform_undock
from core.config_validator import get_validated_config, print_config_summary
from core.safety import SafetyMonitorV2
from core.telemetry import TelemetryLogger
from core.ir_avoid import IRAvoidNavigator

# --- Cargar configuración validada ---
try:
    CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.yaml")
    config = get_validated_config(CONFIG_PATH)
    print_config_summary(config)
except ValueError as e:
    print(f"❌ Error de configuración: {e}")
    sys.exit(1)

robot = Create3(Bluetooth(config['robot']['name']))
cmdq = queue.Queue()
connected = threading.Event()

# --- Estado global ---
origin_mode = {"type": None, "node": None}
pending_origin = {"type": None, "node": None}
nav_mode = "direct"
current_pose = [0.0, 0.0, 0.0]
_safety: Optional[SafetyMonitorV2] = None
_telemetry: Optional[TelemetryLogger] = None
current_nav_task: Optional[asyncio.Task] = None

class NavigationGUI:
    """GUI principal de navegación"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Create3 Navigation System")
        self.root.geometry("1000x700")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Variables
        self.nodes_var = tk.StringVar()
        self.edges_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Desconectado")
        self.pose_var = tk.StringVar(value="Pose: [0.0, 0.0, 0.0°]")

        self.setup_ui()
        self.update_nodes_list()
        self.update_edges_list()

    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

        # === PANEL IZQUIERDO: NODOS Y RUTAS ===
        left_frame = ttk.LabelFrame(main_frame, text="Nodos y Rutas", padding="5")
        left_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))

        ttk.Label(left_frame, text="Nodos:").grid(row=0, column=0, sticky=tk.W)
        self.nodes_listbox = tk.Listbox(left_frame, height=8, width=30)
        self.nodes_listbox.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 5))

        self.search_entry = ttk.Entry(left_frame, width=20)
        self.search_entry.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        ttk.Button(left_frame, text="Buscar", command=self.search_nodes).grid(row=2, column=1, sticky=tk.E)

        ttk.Button(left_frame, text="Actualizar", command=self.update_nodes_list).grid(row=3, column=0, sticky=tk.W)
        ttk.Button(left_frame, text="Ir a Nodo", command=self.go_to_selected_node).grid(row=3, column=1, sticky=tk.E)

        ttk.Label(left_frame, text="Rutas:").grid(row=4, column=0, sticky=tk.W, pady=(10, 0))
        self.edges_listbox = tk.Listbox(left_frame, height=6, width=30)
        self.edges_listbox.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 5))

        ttk.Button(left_frame, text="Actualizar", command=self.update_edges_list).grid(row=6, column=0, sticky=tk.W)
        ttk.Button(left_frame, text="Ver Faltantes", command=self.show_missing_routes).grid(row=6, column=1, sticky=tk.E)

        # === PANEL CENTRAL: CONTROL DE NAVEGACIÓN ===
        center_frame = ttk.LabelFrame(main_frame, text="Control de Navegación", padding="5")
        center_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5)

        ttk.Label(center_frame, text="Estado:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(center_frame, textvariable=self.status_var).grid(row=0, column=1, sticky=tk.W, padx=(5, 0))

        ttk.Label(center_frame, text="Pose:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(center_frame, textvariable=self.pose_var).grid(row=1, column=1, sticky=tk.W, padx=(5, 0))

        origin_frame = ttk.LabelFrame(center_frame, text="Origen", padding="5")
        origin_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)

        ttk.Button(origin_frame, text="Undock", command=self.cmd_undock).grid(row=0, column=0, padx=2)
        ttk.Button(origin_frame, text="Start Dock", command=self.cmd_startdock).grid(row=0, column=1, padx=2)
        ttk.Button(origin_frame, text="Start Nodo", command=self.cmd_start_node).grid(row=0, column=2, padx=2)
        ttk.Button(origin_frame, text="Confirmar", command=self.cmd_confirm_origin).grid(row=1, column=0, padx=2, pady=5)
        ttk.Button(origin_frame, text="Cancelar", command=self.cmd_cancel_origin).grid(row=1, column=1, padx=2, pady=5)

        nav_frame = ttk.LabelFrame(center_frame, text="Navegación", padding="5")
        nav_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)

        ttk.Button(nav_frame, text="Ir a Nodo", command=self.cmd_go_to_node).grid(row=0, column=0, padx=2)
        ttk.Button(nav_frame, text="Ir a Nombre", command=self.cmd_go_to_name).grid(row=0, column=1, padx=2)
        ttk.Button(nav_frame, text="Ir a Origen", command=self.cmd_go_home).grid(row=0, column=2, padx=2)
        ttk.Button(nav_frame, text="Parar", command=self.cmd_stop).grid(row=0, column=3, padx=2)

        mode_frame = ttk.Frame(nav_frame)
        mode_frame.grid(row=1, column=0, columnspan=3, pady=5)
        ttk.Label(mode_frame, text="Modo:").pack(side=tk.LEFT)
        self.mode_var = tk.StringVar(value="direct")
        ttk.Radiobutton(mode_frame, text="Direct", variable=self.mode_var, value="direct").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(mode_frame, text="Replay", variable=self.mode_var, value="replay").pack(side=tk.LEFT, padx=5)

        # === PANEL DERECHO: INFORMACIÓN ===
        right_frame = ttk.LabelFrame(main_frame, text="Información", padding="5")
        right_frame.grid(row=0, column=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))

        ttk.Button(right_frame, text="Ver Config", command=self.show_config).grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Button(right_frame, text="Estado", command=self.show_status).grid(row=1, column=0, sticky=tk.W, pady=2)

        self.conn_label = ttk.Label(right_frame, text="Conexión: ✗", foreground="red")
        self.conn_label.grid(row=2, column=0, sticky=tk.W, pady=2)
        self.safety_label = ttk.Label(right_frame, text="Safety: Off", foreground="orange")
        self.safety_label.grid(row=3, column=0, sticky=tk.W, pady=2)

        safety_frame = ttk.LabelFrame(right_frame, text="Safety", padding="5")
        safety_frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=5)
        ttk.Button(safety_frame, text="Activar Safety", command=self.enable_safety).grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Button(safety_frame, text="Desactivar Safety", command=self.disable_safety).grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Button(safety_frame, text="Override Safety", command=self.override_safety).grid(row=2, column=0, sticky=tk.W, pady=2)

        ttk.Label(right_frame, text="Vecinos:").grid(row=5, column=0, sticky=tk.W, pady=(10, 0))
        self.neighbors_listbox = tk.Listbox(right_frame, height=4, width=25)
        self.neighbors_listbox.grid(row=6, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        ttk.Button(right_frame, text="Ver Vecinos", command=self.show_neighbors).grid(row=7, column=0, sticky=tk.W)

        ttk.Label(right_frame, text="Log:").grid(row=8, column=0, sticky=tk.W, pady=(10, 0))
        self.log_text = tk.Text(right_frame, height=8, width=25)
        self.log_text.grid(row=9, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar = ttk.Scrollbar(right_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        scrollbar.grid(row=9, column=1, sticky=(tk.N, tk.S))
        self.log_text.configure(yscrollcommand=scrollbar.set)

        main_frame.rowconfigure(0, weight=1)
        left_frame.rowconfigure(5, weight=1)
        center_frame.rowconfigure(3, weight=1)
        right_frame.rowconfigure(9, weight=1)
        right_frame.columnconfigure(0, weight=1)

    def log_message(self, message: str):
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)

    def update_nodes_list(self):
        self.nodes_listbox.delete(0, tk.END)
        nodes = load_nodes()
        for node in nodes:
            self.nodes_listbox.insert(tk.END, f"{node['id']}: {node['name']} [{node['x']:.1f}, {node['y']:.1f}]")

    def update_edges_list(self):
        self.edges_listbox.delete(0, tk.END)
        edges = load_edges()
        idx = nodes_index_by_id()
        for edge in edges:
            from_name = idx.get(edge["from"], {"name": "?"})["name"]
            to_name = idx.get(edge["to"], {"name": "?"})["name"]
            self.edges_listbox.insert(tk.END, f"{edge['from']}→{edge['to']}: {from_name}→{to_name}")

    def search_nodes(self):
        query = (self.search_entry.get() or "").strip().lower()
        if not query:
            return
        nodes = load_nodes()
        target_index = None
        for i, n in enumerate(nodes):
            if query == str(n.get("id", "")).lower() or query in n.get("name", "").strip().lower():
                target_index = i
                break
        if target_index is not None:
            self.nodes_listbox.selection_clear(0, tk.END)
            self.nodes_listbox.selection_set(target_index)
            self.nodes_listbox.see(target_index)

    def show_missing_routes(self):
        nodes = load_nodes()
        edges = load_edges()
        missing = compute_missing_routes(nodes, edges)
        if not missing:
            messagebox.showinfo("Rutas Faltantes", "No hay rutas faltantes (grafo completo)")
            return
        idx = nodes_index_by_id()
        missing_text = "Rutas faltantes:\n"
        for (a, b) in missing[:10]:
            na = idx.get(a, {"name": "?"})
            nb = idx.get(b, {"name": "?"})
            missing_text += f"{a}:{na['name']} → {b}:{nb['name']}\n"
        if len(missing) > 10:
            missing_text += f"... y {len(missing)-10} más"
        messagebox.showinfo("Rutas Faltantes", missing_text)

    def show_config(self):
        config_text = "=== CONFIGURACIÓN ACTIVA ===\n"
        config_text += f"Robot: {config['robot']['name']}\n"
        config_text += f"Motion: vel={config['motion']['vel_default_cm_s']} cm/s\n"
        config_text += f"Safety: ir_threshold={config['safety']['ir_threshold']}\n"
        config_text += f"Undock: back={config['undock']['back_cm']} cm, turn={config['undock']['turn_deg']}°"
        messagebox.showinfo("Configuración", config_text)

    def show_status(self):
        global origin_mode, nav_mode, current_pose
        status_text = "=== ESTADO DEL SISTEMA ===\n"
        if origin_mode["type"] is None:
            status_text += "Origen: No definido\n"
        elif origin_mode["type"] == "dock":
            status_text += "Origen: DOCK (0,0)\n"
        else:
            n = origin_mode["node"]
            status_text += f"Origen: Nodo {n['id']}:{n['name']}\n"
        status_text += f"Modo navegación: {nav_mode}\n"
        status_text += f"Pose actual: [{current_pose[0]:.1f}, {current_pose[1]:.1f}, {current_pose[2]:.1f}°]"
        messagebox.showinfo("Estado", status_text)

    def show_neighbors(self):
        selection = self.nodes_listbox.curselection()
        if not selection:
            messagebox.showwarning("Selección", "Selecciona un nodo primero")
            return
        node_text = self.nodes_listbox.get(selection[0])
        node_id = int(node_text.split(':')[0])
        neighbors = neighbors_of(node_id)
        if not neighbors:
            messagebox.showinfo("Vecinos", f"No hay rutas salientes desde nodo {node_id}")
            return
        idx = nodes_index_by_id()
        neighbors_text = f"Rutas desde nodo {node_id}:\n"
        for edge in neighbors:
            dest = idx.get(edge["to"], {"name": "?"})
            neighbors_text += f"→ {edge['to']}:{dest['name']}\n"
        messagebox.showinfo("Vecinos", neighbors_text)

    def go_to_selected_node(self):
        selection = self.nodes_listbox.curselection()
        if not selection:
            messagebox.showwarning("Selección", "Selecciona un nodo primero")
            return
        node_text = self.nodes_listbox.get(selection[0])
        node_id = int(node_text.split(':')[0])
        self.log_message(f"Navegando a nodo {node_id}")
        cmdq.put({"cmd": "g", "args": [str(node_id)]})

    def cmd_undock(self):
        self.log_message("Ejecutando undock...")
        cmdq.put({"cmd": "undock"})

    def cmd_startdock(self):
        self.log_message("Origen pendiente: DOCK")
        cmdq.put({"cmd": "startdock"})

    def cmd_start_node(self):
        selection = self.nodes_listbox.curselection()
        if not selection:
            messagebox.showwarning("Selección", "Selecciona un nodo primero")
            return
        node_text = self.nodes_listbox.get(selection[0])
        node_id = int(node_text.split(':')[0])
        self.log_message(f"Origen pendiente: nodo {node_id}")
        cmdq.put({"cmd": "start", "args": [str(node_id)]})

    def cmd_confirm_origin(self):
        self.log_message("Confirmando origen...")
        cmdq.put({"cmd": "ok"})

    def cmd_cancel_origin(self):
        self.log_message("Origen pendiente cancelado")
        cmdq.put({"cmd": "cancel"})

    def cmd_go_to_node(self):
        node_id = simpledialog.askstring("Navegación", "ID del nodo destino:")
        if node_id:
            self.log_message(f"Navegando a nodo {node_id}")
            cmdq.put({"cmd": "g", "args": [node_id]})

    def cmd_go_to_name(self):
        node_name = simpledialog.askstring("Navegación", "Nombre del nodo destino:")
        if node_name:
            self.log_message(f"Navegando a nodo '{node_name}'")
            cmdq.put({"cmd": "gn", "args": [node_name]})

    def cmd_go_home(self):
        self.log_message("Volviendo a origen (IR Avoid)...")
        cmdq.put({"cmd": "h"})

    def cmd_stop(self):
        self.log_message("Parando navegación...")
        cmdq.put({"cmd": "stop"})

    def update_pose(self, x: float, y: float, theta: float):
        global current_pose
        current_pose = [x, y, theta]
        self.pose_var.set(f"Pose: [{x:.1f}, {y:.1f}, {theta:.1f}°]")

    def update_status(self, status: str):
        self.status_var.set(status)

    def enable_safety(self):
        self.log_message("Safety: Activando...")
        cmdq.put({"cmd": "enable_safety"})

    def disable_safety(self):
        self.log_message("Safety: Desactivando...")
        cmdq.put({"cmd": "disable_safety"})

    def override_safety(self):
        self.log_message("Safety: Override aplicado")
        cmdq.put({"cmd": "override_safety"})

    def on_closing(self):
        cmdq.put({"cmd": "q"})
        self.root.destroy()


async def read_pose(robot):
    p = await robot.get_position()
    try:
        x, y, th = p.x, p.y, p.heading
    except AttributeError:
        x, y, th = p
    return x, y, th


def _normalize_angle_deg(a):
    while a >= 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


async def cmd_undock(robot):
    global origin_mode
    undock_cfg = config['undock']
    await perform_undock(robot,
                         back_cm=undock_cfg['back_cm'],
                         turn_deg=undock_cfg['turn_deg'],
                         turn_dir=undock_cfg['turn_dir'],
                         back_speed=undock_cfg['back_speed'])
    pose = await read_pose(robot)
    from nodes_io import append_node
    startn = append_node(pose[0], pose[1], pose[2], name="StartFromDock", source="undock")
    origin_mode["type"] = "dock"
    origin_mode["node"] = None
    print(f"✔ Origen DOCK. Nodo StartFromDock id={startn['id']} creado.")


async def cmd_confirm_origin(robot):
    global origin_mode
    if pending_origin["type"] is None:
        print("❌ No hay origen pendiente. Usa 'startdock' o 'start <id|nombre>'.")
        return
    if pending_origin["type"] == "dock":
        print("Coloca el robot en el Dock. Presiona ENTER cuando esté listo.")
        await cmd_undock(robot)
    else:
        n = pending_origin["node"]
        print(f"Coloca el robot en nodo {n['id']}:'{n['name']}' [x={n['x']:.1f}, y={n['y']:.1f}, θ={n['theta']:.1f}]")
        print("Alinea orientación. Presiona ENTER cuando esté listo.")
        await robot.reset_navigation()
        await robot.set_lights_on_rgb(0, 255, 0)
        origin_mode["type"] = "node"
        origin_mode["node"] = n
        print(f"✔ Origen establecido en nodo {n['id']}:'{n['name']}'")
    pending_origin["type"] = None
    pending_origin["node"] = None


async def _start_services():
    global _safety, _telemetry
    _safety = SafetyMonitorV2(robot,
                              ir_threshold=config['safety']['ir_threshold'],
                              period_s=config['safety']['safety_period_s'],
                              front_idx=tuple(config.get('avoidance', {}).get('front_idx', (2,3,4))))
    _safety.enable(config['safety']['enable_auto_brake'])
    await _safety.start()

    _telemetry = TelemetryLogger(robot,
                                 out_dir=config['telemetry']['log_dir'],
                                 period_s=config['telemetry']['period_s'])
    await _telemetry.start()
    print("✔ Servicios iniciados: Safety v2 + Telemetry")


async def _navigate_to_nodes(node_ids: List[int]):
    idx = nodes_index_by_id()
    nav = IRAvoidNavigator(robot, config, safety=_safety, telemetry=_telemetry)

    # Respetar configuración de auto-brake (no forzar ON si está deshabilitado)
    try:
        if _safety and config['safety'].get('enable_auto_brake', False):
            _safety.enable(True)
            await _safety.start()
    except Exception:
        pass

    for nid in node_ids:
        dest = idx.get(nid)
        if not dest:
            print(f"❌ Nodo {nid} no encontrado.")
            continue
        print(f"→ Navegación IR hacia nodo {nid}:{dest['name']} @ ({dest['x']:.1f},{dest['y']:.1f})")
        start_pose = await read_pose(robot)
        try:
            ok, end_pose = await nav.go_to(dest['x'], dest['y'])
        except asyncio.CancelledError:
            await robot.set_wheel_speeds(0, 0)
            print("⏹ Navegación cancelada.")
            return
        # Log intento
        log_nav_attempt(
            target=f"{nid}:{dest['name']}",
            plan_x=dest['x'], plan_y=dest['y'], plan_theta=None,
            start_pose=start_pose,
            end_pose=end_pose,
            origin_info={"type": ("dock" if origin_mode["type"] == "dock" else "node"),
                         "node": origin_mode.get("node")}
        )
        if ok:
            print(f"✔ Llegada a nodo {nid}:{dest['name']}")
            try:
                await robot.play_note(659, 0.12)
            except Exception:
                pass
        else:
            print(f"⚠ No se alcanzó el nodo {nid} en el tiempo límite.")


@event(robot.when_play)
async def main_loop(robot):
    global origin_mode, pending_origin, nav_mode, current_pose, _safety, _telemetry, current_nav_task

    if not connected.is_set():
        connected.set()
        print("✔ Robot conectado.")
        await _start_services()

    while True:
        if not cmdq.empty():
            cmd_data = cmdq.get_nowait()
            cmd = cmd_data.get("cmd", "")
            args = cmd_data.get("args", [])

            if cmd == "undock":
                await cmd_undock(robot)

            elif cmd == "startdock":
                pending_origin["type"] = "dock"
                pending_origin["node"] = None
                print("Origen pendiente: DOCK. Usa 'ok' para confirmar.")

            elif cmd == "start":
                if args:
                    n = resolve_node(args[0])
                    if not n:
                        print(f"❌ Nodo '{args[0]}' no encontrado.")
                    else:
                        pending_origin["type"] = "node"
                        pending_origin["node"] = n
                        print(f"Origen pendiente: nodo {n['id']}:'{n['name']}'. Usa 'ok' para confirmar.")

            elif cmd == "ok":
                await cmd_confirm_origin(robot)

            elif cmd == "cancel":
                pending_origin["type"] = None
                pending_origin["node"] = None
                print("Origen pendiente cancelado.")

            elif cmd == "g":
                if args:
                    node_ids = []
                    for a in args:
                        try:
                            node_ids.append(int(a))
                        except:
                            print(f"❌ '{a}' no es un ID válido.")
                            node_ids = []
                            break
                    if node_ids:
                        if current_nav_task and not current_nav_task.done():
                            current_nav_task.cancel()
                            try:
                                await current_nav_task
                            except Exception:
                                pass
                        current_nav_task = asyncio.create_task(_navigate_to_nodes(node_ids))

            elif cmd == "gn":
                if args:
                    n = resolve_node(args[0])
                    if not n:
                        print(f"❌ Nodo '{args[0]}' no encontrado.")
                    else:
                        if current_nav_task and not current_nav_task.done():
                            current_nav_task.cancel()
                            try:
                                await current_nav_task
                            except Exception:
                                pass
                        current_nav_task = asyncio.create_task(_navigate_to_nodes([n['id']]))

            elif cmd == "h":
                # Ir a origen (0,0) con evasión IR
                print("Volviendo a origen con IR Avoid...")
                if current_nav_task and not current_nav_task.done():
                    current_nav_task.cancel()
                    try:
                        await current_nav_task
                    except Exception:
                        pass
                try:
                    if _safety and config['safety'].get('enable_auto_brake', False):
                        _safety.enable(True)
                        await _safety.start()
                except Exception:
                    pass
                nav = IRAvoidNavigator(robot, config, safety=_safety, telemetry=_telemetry)
                ok, _ = await nav.go_to(0.0, 0.0)
                if ok:
                    print("✔ En origen (0,0).")
                else:
                    print("⚠ No fue posible alcanzar origen dentro del tiempo.")

            elif cmd == "stop":
                if current_nav_task and not current_nav_task.done():
                    current_nav_task.cancel()
                    try:
                        await current_nav_task
                    except Exception:
                        pass
                await robot.set_wheel_speeds(0, 0)
                print("⏹ Navegación detenida por usuario.")

            elif cmd == "q":
                print("Saliendo...")
                break

            elif cmd == "enable_safety":
                try:
                    if _safety:
                        _safety.enable(True)
                        await _safety.start()
                        print("✓ Safety habilitado")
                except Exception as e:
                    print(f"(enable_safety) fallo: {e}")

            elif cmd == "disable_safety":
                try:
                    if _safety:
                        _safety.enable(False)
                        print("✓ Safety deshabilitado")
                except Exception as e:
                    print(f"(disable_safety) fallo: {e}")

            elif cmd == "override_safety":
                try:
                    if _safety:
                        await _safety.clear_halt()
                except Exception as e:
                    print(f"(override_safety) fallo: {e}")

        # Actualizar pose
        try:
            pose = await read_pose(robot)
            current_pose = list(pose)
        except:
            pass

        await robot.wait(0.1)


def gui_thread():
    gui = NavigationGUI()

    def update_pose():
        try:
            gui.update_pose(current_pose[0], current_pose[1], current_pose[2])
        except:
            pass
        try:
            gui.conn_label.configure(text=("Conexión: ✓" if connected.is_set() else "Conexión: ✗"),
                                     foreground=("green" if connected.is_set() else "red"))
            if _safety and _safety.enabled:
                fg = "red" if _safety.halted.is_set() else "green"
                txt = "Safety: Halt" if _safety.halted.is_set() else "Safety: On"
            else:
                fg = "orange"; txt = "Safety: Off"
            gui.safety_label.configure(text=txt, foreground=fg)
        except:
            pass
        gui.root.after(1000, update_pose)

    update_pose()
    gui.root.mainloop()


if __name__ == "__main__":
    print("=== Sistema de Navegación Create3 - GUI ===")
    print("Conectando...")

    t_robot = threading.Thread(target=robot.play, daemon=True)
    t_robot.start()

    if not connected.wait(timeout=20):
        print("❌ No se pudo conectar al robot.")
        sys.exit(1)

    t_gui = threading.Thread(target=gui_thread, daemon=True)
    t_gui.start()

    try:
        t_gui.join()
    except KeyboardInterrupt:
        pass

    print("Fin.")
