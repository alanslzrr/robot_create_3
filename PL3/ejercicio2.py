"""
PRÁCTICA DE LABORATORIO 3 - Conocer el Entorno y Aprender
================================================================================
ROBOTS AUTÓNOMOS - Ejercicio 3.2 «Entorno con información previa»

INFORMACIÓN BÁSICA:
- Autores: Yago Ramos - Salazar Alan
- Fecha de finalización: 14 de octubre de 2025 
- Institución: UIE
- Robot SDK: irobot-edu-sdk
- Requiere: Haber ejecutado ejercicio1.py y generado T03_Entorno.jsonl

OBJETIVO ESPECÍFICO:
Incorporar la lectura del fichero de entorno previo para optimizar el recorrido
del robot, evitando zonas ya exploradas y asegurando consistencia entre la 
información almacenada y el comportamiento del robot.

COMPORTAMIENTO ESPERADO:
1. Carga automáticamente el fichero T03_Entorno.jsonl generado en ejercicio 1
2. Durante la exploración, verifica si las zonas proyectadas ya fueron visitadas
3. Prioriza rutas hacia áreas desconocidas sobre zonas ya exploradas
4. Utiliza predicción geométrica para evaluar celdas tras giros de 90°
5. Finaliza cuando no quedan áreas nuevas accesibles (optimización completa)

SENSORES UTILIZADOS:
- Sensores IR de proximidad (array de 7 sensores)
  * ir[3]: Sensor frontal principal (detección de obstáculos)
  * ir[0], ir[1]: Sensores laterales izquierdos (evaluación de rutas)
  * ir[5], ir[6]: Sensores laterales derechos (evaluación de rutas)
- Sistema de odometría (posición x,y y orientación θ)

ESTRATEGIA DE OPTIMIZACIÓN:
- Proyección geométrica: Calcula posición tras giro de 90° (±10cm)
- Verificación de conocimiento: Consulta si posición proyectada ya está en mapa
- Decisión inteligente: Prioriza rutas hacia zonas desconocidas
- Evitación de redundancia: No explora áreas ya mapeadas

CRITERIOS DE DECISIÓN:
1. Si ambas rutas (izq/der) están bloqueadas o son conocidas → FINALIZAR
2. Si solo una ruta es desconocida → Tomar esa dirección
3. Si ambas son desconocidas → Preferir izquierda (heurística)
4. Si ambas son conocidas pero accesibles → Tomar dirección menos explorada

ESTRUCTURAS DE DATOS:
- Place: Reutiliza estructura del ejercicio 1
- MapManager: Versión simplificada para lectura y consulta rápida
- Predicción geométrica: Cálculo trigonométrico de posiciones futuras

ALMACENAMIENTO:
- Lectura: Carga completa del fichero T03_Entorno.jsonl
- Consulta: Búsqueda eficiente por proximidad espacial
- Persistencia: No modifica el fichero (solo lectura)

PARÁMETROS DE CONFIGURACIÓN:
- IR_OBS_THRESHOLD: 120 (~15cm para detección frontal)
- IR_DIR_THRESHOLD: 200 (umbral para considerar bloqueo lateral)
- POS_EPS: 5.0cm (radio para considerar posición ya visitada)
- PROYECCIÓN_STEP: 10cm (distancia proyectada tras giro)
"""

from __future__ import annotations
import json, math
from pathlib import Path
from dataclasses import dataclass
from typing import List, Literal

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# ── constantes ────────────────────────────────────────────────────────────
FILE_NAME        = Path("T03_Entorno.jsonl")
EDGES_FILE_NAME  = Path("T03_Entorno_edges.jsonl")  # reproducir sin sensores
IR_OBS_THRESHOLD = 120
IR_DIR_THRESHOLD = 200
POS_EPS          = 5.0

# ── estructuras ───────────────────────────────────────────────────────────
@dataclass
class Place:
    id: int; x: float; y: float; theta: float
    ir_front: int; ir_left: int; ir_right: int; timestamp: str
    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)

class MapManager:
    def __init__(self, path: Path):
        if not path.exists():
            raise FileNotFoundError(
                "Debe ejecutar primero ejercicio3_1.py para generar el mapa.")
        self.places: List[Place] = []
        with path.open() as f:
            for line in f:
                if line.strip():
                    self.places.append(Place(**json.loads(line)))

    def find_near(self, x: float, y: float) -> bool:
        return any(p.distance_to(x, y) < POS_EPS for p in self.places)

@dataclass
class Edge:
    from_id: int
    to_id: int
    turn: Literal['left','right','straight']
    segment_cm: float
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    timestamp: str

class EdgeManager:
    def __init__(self, path: Path):
        self.edges: List[Edge] = []
        if not path.exists():
            # Ejecutable incluso si no existen aristas aún
            return
        with path.open() as f:
            for line in f:
                if line.strip():
                    self.edges.append(Edge(**json.loads(line)))

# ── utilidades ────────────────────────────────────────────────────────────
def ir_left(ir):  return max(ir[0], ir[1])
def ir_right(ir): return max(ir[5], ir[6])

async def wait_for_front_obstacle(rbt):
    while True:
        ir = (await rbt.get_ir_proximity()).sensors
        if ir[3] > IR_OBS_THRESHOLD:
            return ir

# ── control de movimiento para replay ──────────────────────────────────────
def _normalize_angle_deg(angle: float) -> float:
    """Normaliza ángulo a (-180, 180]."""
    return (angle + 180.0) % 360.0 - 180.0

async def drive_forward_pid(rbt, distance_cm: float, base_speed: float = 8.0, kp: float = 0.6, timeout_s: float = 20.0):
    """Avanza en línea recta 'distance_cm' manteniendo el heading inicial con control proporcional.
    - base_speed en cm/s por rueda
    - kp en (cm/s)/deg para corrección diferencial por error de heading
    - timeout de seguridad
    """
    start = await rbt.get_position()
    start_x, start_y, start_heading = start.x, start.y, start.heading
    elapsed = 0.0
    dt = 0.05
    while True:
        pos = await rbt.get_position()
        dx, dy = pos.x - start_x, pos.y - start_y
        dist = math.hypot(dx, dy)
        if dist >= max(0.0, distance_cm - 0.5):  # margen 0.5 cm para evitar sobrepaso
            break
        err_deg = _normalize_angle_deg(start_heading - pos.heading)
        correction = kp * err_deg
        left = base_speed - correction
        right = base_speed + correction
        # Saturación simple para evitar comandos excesivos
        max_speed = 20.0
        left = max(-max_speed, min(max_speed, left))
        right = max(-max_speed, min(max_speed, right))
        await rbt.set_wheel_speeds(left, right)
        await rbt.wait(dt)
        elapsed += dt
        if elapsed > timeout_s:
            print("   Aviso: timeout avanzando, deteniendo por seguridad")
            break
    # Frenado
    await rbt.set_wheel_speeds(0, 0)

# ── robot ─────────────────────────────────────────────────────────────────
robot        = Create3(Bluetooth("C3_UIEC_Grupo1"))
map_manager  = MapManager(FILE_NAME)
edge_manager = EdgeManager(EDGES_FILE_NAME)

@event(robot.when_play)
async def play(rbt):
    """Recorrido optimizado usando la memoria previa.

    Modo de operación:
    - Si existen aristas en EDGES_FILE_NAME (generadas por ejercicio1.py), se ejecuta
      un "replay" sin sensores: homing inicial hasta obstáculo (para llegar a Place 0),
      luego por cada arista se realiza un giro ±90° y un avance recto controlado por
      odometría con estabilización de rumbo.
    - Si no existen aristas, cae al comportamiento original basado en sensores IR y
      proyección geométrica, manteniendo compatibilidad con el enfoque inicial.

    Leyenda de LEDs (ejercicio 2):
    - Azul (0,0,255): Estado por defecto (avance y espera).
    - Rojo (255,0,0): Justo durante el giro (tanto en replay como en modo sensores).
    - Verde (0,255,0): Finalización del recorrido.

    Sonidos:
    - Beep breve en cada giro durante el modo replay.
    """
    print(f"CARGANDO MAPA PREVIO: {len(map_manager.places)} lugares conocidos")
    for i, place in enumerate(map_manager.places[:3]):  # Muestra primeros 3 lugares
        print(f"   Lugar {i}: ({place.x:.1f}, {place.y:.1f}) θ={place.theta:.1f}°")
    if len(map_manager.places) > 3:
        print(f"   ... y {len(map_manager.places) - 3} lugares más")
    if edge_manager.edges:
        print(f"CARGADAS ARISTAS PARA REPLAY: {len(edge_manager.edges)} tramos")
        for e in edge_manager.edges[:3]:
            print(f"   {e.from_id} --{e.turn}/{e.segment_cm:.1f}cm--> {e.to_id}")
        if len(edge_manager.edges) > 3:
            print(f"   ... y {len(edge_manager.edges)-3} tramos más")

    await rbt.reset_navigation()
    # Color de operación inicial: AZUL (avance)
    await rbt.set_lights_on_rgb(0, 0, 255)
    step_count = 0

    # Modo 1: Si hay aristas, ejecuta reproducción sin sensores.
    if edge_manager.edges:
        print("\nMODO REPLAY SIN SENSORES (usando aristas)")
        # Pre-move sin sensores: navegar hasta el Place 0 usando solo odometría
        pos0 = await rbt.get_position()
        print(f"   Pose inicial: ({pos0.x:.1f}, {pos0.y:.1f}) θ={pos0.heading:.1f}°")
        try:
            first_from_id = edge_manager.edges[0].from_id
            place0 = map_manager.places[first_from_id]
        except Exception:
            place0 = map_manager.places[0]
        dx0 = place0.x - pos0.x
        dy0 = place0.y - pos0.y
        dist0 = math.hypot(dx0, dy0)
        if dist0 > 0.5:
            desired_bearing = math.degrees(math.atan2(dy0, dx0))
            delta = _normalize_angle_deg(desired_bearing - pos0.heading)
            if abs(delta) > 1.0:
                if delta > 0:
                    await rbt.turn_left(abs(delta))
                else:
                    await rbt.turn_right(abs(delta))
                print(f"   Alinear a Place 0: rotar Δ={delta:.1f}° hacia {desired_bearing:.1f}°")
            print(f"   Avanzar hasta Place 0: d0={dist0:.1f} cm -> ({place0.x:.1f}, {place0.y:.1f})")
            await drive_forward_pid(rbt, dist0, base_speed=10.0, kp=0.6, timeout_s=40.0)
            # Alinear heading al del Place 0
            pos0b = await rbt.get_position()
            d_heading = _normalize_angle_deg(place0.theta - pos0b.heading)
            if abs(d_heading) > 1.0:
                if d_heading > 0:
                    await rbt.turn_left(abs(d_heading))
                else:
                    await rbt.turn_right(abs(d_heading))
                print(f"   Alinear heading a Place 0: Δ={d_heading:.1f}° -> θ0={place0.theta:.1f}°")
        else:
            print("   Ya en Place 0 (desplazamiento < 0.5 cm)")
        for idx, e in enumerate(edge_manager.edges, 1):
            print(f"Paso {idx}: ID {e.from_id} -> {e.to_id} ({e.turn}, {e.segment_cm:.1f} cm)")
            # Giro preciso ±90° (o ningún giro si es 'straight')
            if e.turn == 'left':
                await rbt.set_lights_on_rgb(255, 0, 0)
                await rbt.turn_left(90)
                await rbt.set_lights_on_rgb(0, 0, 255)
                await rbt.play_note(784, 0.2)
            elif e.turn == 'right':
                await rbt.set_lights_on_rgb(255, 0, 0)
                await rbt.turn_right(90)
                await rbt.set_lights_on_rgb(0, 0, 255)
                await rbt.play_note(784, 0.2)
            else:
                # 'straight' → no realizar giro previo
                await rbt.set_lights_on_rgb(0, 0, 255)
            # Avance con estabilización de heading (PID sencillo)
            await drive_forward_pid(rbt, e.segment_cm, base_speed=10.0, kp=0.6, timeout_s=30.0)
        print("\nREPLAY COMPLETADO")
        # Final de replay: color VERDE + sonido de cierre
        await rbt.set_lights_on_rgb(0, 255, 0)
        await rbt.play_note(523, 0.6)
        print(f"   Tramos ejecutados: {len(edge_manager.edges)}")
        return

    # Modo 2: Comportamiento original (con sensores) si no hay aristas
    while True:
        step_count += 1
        print(f"\nPASO {step_count} - EXPLORACIÓN OPTIMIZADA")
        
        # 1. Avanza hasta obstáculo (AZUL)
        await rbt.set_lights_on_rgb(0, 0, 255)
        print("   Avanzando hasta detectar obstáculo frontal...")
        await rbt.set_wheel_speeds(5, 5)
        ir = await wait_for_front_obstacle(rbt)
        await rbt.set_wheel_speeds(0, 0)
        pos = await rbt.get_position()
        
        print(f"   Posición actual: ({pos.x:.1f}, {pos.y:.1f}) θ={pos.heading:.1f}°")
        print(f"   Sensores IR: Front={ir[3]}, Left={ir_left(ir)}, Right={ir_right(ir)}")

        # 2. Analiza bloqueos y si las celdas tras giro ya son conocidas
        left_blocked  = ir_left(ir)  > IR_DIR_THRESHOLD
        right_blocked = ir_right(ir) > IR_DIR_THRESHOLD
        
        print(f"   Bloqueos: Izquierda={'SÍ' if left_blocked else 'NO'}, Derecha={'SÍ' if right_blocked else 'NO'}")

        step = 10  # cm proyectados después del giro
        left_known = right_known = False
        
        # Predicción geométrica para izquierda (heading + 90°)
        if not left_blocked:
            ang = math.radians(pos.heading + 90)
            proj_x = pos.x + step*math.cos(ang)
            proj_y = pos.y + step*math.sin(ang)
            left_known = map_manager.find_near(proj_x, proj_y)
            print(f"   Proyección IZQUIERDA: ({proj_x:.1f}, {proj_y:.1f}) -> {'CONOCIDA' if left_known else 'DESCONOCIDA'}")
        
        # Predicción geométrica para derecha  (heading - 90°)
        if not right_blocked:
            ang = math.radians(pos.heading - 90)
            proj_x = pos.x + step*math.cos(ang)
            proj_y = pos.y + step*math.sin(ang)
            right_known = map_manager.find_near(proj_x, proj_y)
            print(f"   Proyección DERECHA: ({proj_x:.1f}, {proj_y:.1f}) -> {'CONOCIDA' if right_known else 'DESCONOCIDA'}")

        # 3. Decisión de giro
        print("   ANÁLISIS DE DECISIÓN:")
        
        if (left_blocked or left_known) and (right_blocked or right_known):
            print("   AMBAS RUTAS: Bloqueadas o ya exploradas -> FINALIZAR")
            # Sin salidas nuevas: VERDE + tono
            await rbt.set_lights_on_rgb(0, 255, 0)
            await rbt.play_note(523, 0.5)   # sin áreas nuevas
            break
        elif not left_blocked and not left_known:
            print("   IZQUIERDA: Libre y desconocida -> GIRAR IZQUIERDA")
            # Indicar giro con ROJO
            await rbt.set_lights_on_rgb(255, 0, 0)
            await rbt.turn_left(90)
            # Tras decidir, volver a AZUL de avance
            await rbt.set_lights_on_rgb(0, 0, 255)
        elif not right_blocked and not right_known:
            print("   DERECHA: Libre y desconocida -> GIRAR DERECHA")
            await rbt.set_lights_on_rgb(255, 0, 0)
            await rbt.turn_right(90)
            await rbt.set_lights_on_rgb(0, 0, 255)
        elif not left_blocked:
            print("   IZQUIERDA: Libre pero conocida -> GIRAR IZQUIERDA")
            await rbt.set_lights_on_rgb(255, 0, 0)
            await rbt.turn_left(90)
            await rbt.set_lights_on_rgb(0, 0, 255)
        else:
            print("   DERECHA: Libre pero conocida -> GIRAR DERECHA")
            await rbt.set_lights_on_rgb(255, 0, 0)
            await rbt.turn_right(90)
            await rbt.set_lights_on_rgb(0, 0, 255)

    print(f"\nRECORRIDO OPTIMIZADO FINALIZADO")
    print(f"   Total de pasos ejecutados: {step_count}")
    print(f"   Mapa previo utilizado: {len(map_manager.places)} lugares")
    print(f"   No quedan zonas nuevas accesibles")

if __name__ == "__main__":
    robot.play()