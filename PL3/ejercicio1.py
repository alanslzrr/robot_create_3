"""
PRÁCTICA DE LABORATORIO 3 - Conocer el Entorno y Aprender
================================================================================
ROBOTS AUTÓNOMOS - Ejercicio 3.1 «Conocer el Entorno»

INFORMACIÓN BÁSICA:
- Autores: Yago Ramos - Salazar Alan
- Fecha de finalización: 14 de octubre de 2025 
- Institución: UIE
- Robot SDK: irobot-edu-sdk

OBJETIVO ESPECÍFICO:
Implementar un sistema de exploración autónoma que registre lugares visitados
mediante sensores IR, diseñe estructuras de datos apropiadas y genere un 
almacenamiento persistente del entorno explorado.

COMPORTAMIENTO ESPERADO:
1. El robot avanza recto hasta detectar un obstáculo frontal
2. Al detectar obstáculo, registra la posición actual y lecturas de sensores
3. Evalúa alternativas de giro (izquierda/derecha) basado en sensores laterales
4. Continúa exploración hasta agotar todas las rutas posibles
5. Genera fichero T03_Entorno.jsonl con secuencia completa de lugares

SENSORES UTILIZADOS:
- Sensores IR de proximidad (array de 7 sensores)
  * ir[3]: Sensor frontal principal (detección de obstáculos)
  * ir[0], ir[1]: Sensores laterales izquierdos (decisión de giro)
  * ir[5], ir[6]: Sensores laterales derechos (decisión de giro)
- Sistema de odometría (posición x,y y orientación θ)

ESTRUCTURAS DE DATOS:
- Place: Representa un punto de interés con posición, orientación y datos sensoriales
- MapManager: Gestiona carga/guardado persistente en formato JSON-Lines

ALMACENAMIENTO PERSISTENTE:
- Formato: JSON-Lines (.jsonl) para escritura incremental
- Ubicación: T03_Entorno.jsonl
- Datos: ID, coordenadas (x,y,θ), lecturas IR, timestamp ISO-UTC

CRITERIOS DE FINALIZACIÓN:
- No quedan rutas accesibles (ambos lados bloqueados por obstáculos)
- Detección basada en umbrales IR_DIR_THRESHOLD para sensores laterales

PARÁMETROS DE CONFIGURACIÓN:
- IR_OBS_THRESHOLD: 120 (~15cm para detección frontal)
- IR_DIR_THRESHOLD: 200 (umbral para considerar bloqueo lateral)
- POS_EPS: 5.0cm (radio para considerar posición ya visitada)
"""

# ╭───────────────────────────  IMPORTACIONES  ─────────────────────────────╮
from __future__ import annotations
import json, math, time
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Optional

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3
# ╰─────────────────────────────────────────────────────────────────────────╯

# ╭─────────────────────────────  CONSTANTES  ──────────────────────────────╮
# Ficheros de persistencia:
# - FILE_NAME:        Secuencia de "lugares" (nodos) detectados al detenerse por obstáculo
# - EDGES_FILE_NAME:  Secuencia de "aristas" (giro elegido + distancia entre lugares consecutivos)
# Umbrales de sensores IR y tolerancias espaciales de deduplicación de lugares.
FILE_NAME           = Path("T03_Entorno.jsonl")  # persistencia línea-a-línea
EDGES_FILE_NAME     = Path("T03_Entorno_edges.jsonl")  # plan de reproducción (giro + tramo)
IR_OBS_THRESHOLD    = 120    # ≈ 15 cm obstáculo frontal
IR_DIR_THRESHOLD    = 200    # umbral giro (bloqueo lateral)
POS_EPS             = 5.0    # cm; radio máx. para considerar “ya visitado”
# ╰─────────────────────────────────────────────────────────────────────────╯

# ╭─────────────────────────  ESTRUCTURAS DE DATOS  ────────────────────────╮
@dataclass
class Place:
    """Representa un punto de interés registrado durante la exploración."""
    id:        int
    x:         float
    y:         float
    theta:     float            # orientación (°)
    ir_front:  int
    ir_left:   int
    ir_right:  int
    timestamp: str              # ISO-UTC

    # ── utilidades ────────────────────────────────────────────────────────
    def distance_to(self, x: float, y: float) -> float:
        """Distancia Euclídea al punto (x,y) en centímetros."""
        return math.hypot(self.x - x, self.y - y)

    def __repr__(self) -> str:  # facilita depuración
        return (f"<Place #{self.id} ({self.x:.1f},{self.y:.1f}) θ={self.theta:.1f}° "
                f"IR[L={self.ir_left},F={self.ir_front},R={self.ir_right}]>")
# ╰─────────────────────────────────────────────────────────────────────────╯

# ╭────────────────────────────  MAPA / I/O  ───────────────────────────────╮
# Gestiona la carga y persistencia incremental del fichero JSON-Lines de lugares.
class MapManager:
    """Carga/guarda una colección de lugares en formato JSON-Lines."""
    def __init__(self, path: Path):
        self.path   = path
        self.places: List[Place] = []
        self._load()

    # ── lectura inicial ───────────────────────────────────────────────────
    def _load(self) -> None:
        if not self.path.exists():
            return                                # primera ejecución: fichero vacío
        with self.path.open() as f:
            for line in f:
                if line.strip():
                    self.places.append(Place(**json.loads(line)))

    # ── escritura incremental ────────────────────────────────────────────
    def append(self, place: Place) -> None:
        """Añade `place` al mapa y lo persiste inmediatamente."""
        self.places.append(place)
        with self.path.open("a") as f:
            json.dump(asdict(place), f)
            f.write("\n")

    # ── consulta rápida ───────────────────────────────────────────────────
    def find_near(self, x: float, y: float) -> bool:
        """True si (x,y) ya está cubierto por otro lugar (radio POS_EPS)."""
        return any(p.distance_to(x, y) < POS_EPS for p in self.places)

    def find_near_place(self, x: float, y: float) -> Optional[Place]:
        """Devuelve el `Place` más cercano dentro de POS_EPS, si existe."""
        best: Optional[Place] = None
        best_d = float("inf")
        for p in self.places:
            d = p.distance_to(x, y)
            if d < POS_EPS and d < best_d:
                best = p
                best_d = d
        return best
# ╰─────────────────────────────────────────────────────────────────────────╯

# ╭────────────────────────────  ARISTAS / I-O  ────────────────────────────╮
# El grafo dirigido se registra como lista de aristas, cada una con:
#  - from_id → to_id: lugares consecutivos
#  - turn: 'left'/'right' (decisión tomada antes del tramo)
#  - segment_cm: distancia odométrica entre ambos lugares
#  - start_x/y, end_x/y: posiciones absolutas en cm (útil para depuración)
@dataclass
class Edge:
    """Une dos lugares consecutivos con la decisión tomada y la longitud del tramo."""
    from_id:   int
    to_id:     int
    turn:      str      # 'left' | 'right' | 'straight'
    segment_cm: float
    start_x:   float
    start_y:   float
    end_x:     float
    end_y:     float
    timestamp: str

class EdgeManager:
    """Persistencia incremental de aristas en JSON-Lines."""
    def __init__(self, path: Path):
        self.path  = path
        self.edges: List[Edge] = []
        self._load()

    def _load(self) -> None:
        if not self.path.exists():
            return
        with self.path.open() as f:
            for line in f:
                if line.strip():
                    self.edges.append(Edge(**json.loads(line)))

    def append(self, edge: Edge) -> None:
        self.edges.append(edge)
        with self.path.open("a") as f:
            json.dump(asdict(edge), f)
            f.write("\n")
# ╰─────────────────────────────────────────────────────────────────────────╯

# ╭────────────────────────────  UTILIDADES  ───────────────────────────────╮
def ir_left(ir):  return max(ir[0], ir[1])           # útil para decidir giro
def ir_right(ir): return max(ir[5], ir[6])

async def wait_for_front_obstacle(rbt):
    """Bloquea hasta que el sensor frontal detecta obstáculo < 15 cm."""
    while True:
        ir = (await rbt.get_ir_proximity()).sensors
        if ir[3] > IR_OBS_THRESHOLD:
            return ir
# ╰─────────────────────────────────────────────────────────────────────────╯

# ╭──────────────────────────  CONFIG. DEL ROBOT  ──────────────────────────╮
robot = Create3(Bluetooth("C3_UIEC_Grupo1"))
# ╰─────────────────────────────────────────────────────────────────────────╯

# ╭──────────────────────────  LÓGICA PRINCIPAL  ───────────────────────────╮
@event(robot.when_play)
async def play(rbt):
    """Explora el entorno y almacena lugares hasta quedar sin salidas.

    Leyenda de LEDs (feedback de estado):
    - Azul   (0,0,255): Avance/crucero normal mientras busca obstáculo frontal.
    - Rojo   (255,0,0): Obstáculo detectado; robot detiene y prepara inspección.
    - Amarillo (255,255,0): Fase de inspección/decisión de giro (izq./der.).
    - Verde  (0,255,0): Fin de exploración (sin salidas disponibles).

    Notas:
    - Tras completar el giro, se restaura Azul para retomar el avance.
    - Se emite un beep breve cuando se registra un nuevo lugar.
    """
    print("INICIANDO EXPLORACIÓN AUTÓNOMA")
    print(f"Archivo de mapa: {FILE_NAME}")
    print(f"Archivo de aristas: {EDGES_FILE_NAME}")
    # Luz de navegación: azul
    await rbt.set_lights_on_rgb(0, 0, 255)
    
    await rbt.reset_navigation()
    # Registrar ORIGEN explícito si no existe aún (o si no hay un lugar cercano al arranque)
    pos0 = await rbt.get_position()
    ir0 = (await rbt.get_ir_proximity()).sensors
    step_idx = len(map_manager.places)  # continúa numeración si existe mapa
    step_count = 0
    last_place: Optional[Place]
    last_turn: Optional[str]
    if step_idx == 0 or map_manager.find_near(pos0.x, pos0.y) is None:
        origin = Place(
            id=step_idx,
            x=pos0.x, y=pos0.y, theta=pos0.heading,
            ir_front=ir0[3], ir_left=ir_left(ir0), ir_right=ir_right(ir0),
            timestamp=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        )
        map_manager.append(origin)
        print(f"ORIGEN REGISTRADO: ID={origin.id} -> ({origin.x:.1f}, {origin.y:.1f}) θ={origin.theta:.1f}°")
        step_idx += 1
        last_place = origin
        # Marcar tramo inicial como 'straight' para poder registrar la arista
        # origen -> primer obstáculo cuando se detecte.
        last_turn = 'straight'
    else:
        # Si ya existía mapa, anclar al lugar más cercano al arranque
        near = map_manager.find_near_place(pos0.x, pos0.y)
        last_place = near if near is not None else (map_manager.places[-1] if map_manager.places else None)
        last_turn = None

    while True:
        step_count += 1
        print(f"\nPASO {step_count} - EXPLORACIÓN BÁSICA")
        
        # ── 1. Avanza recto hasta obstáculo ───────────────────────────────
        # Se desplaza a velocidad baja y bloquea hasta que el IR frontal supere el umbral.
        print("   Avanzando hasta detectar obstáculo frontal...")
        # Color de avance: azul
        await rbt.set_lights_on_rgb(0, 0, 255)
        await rbt.set_wheel_speeds(5, 5)
        ir = await wait_for_front_obstacle(rbt)
        # Obstáculo detectado: ROJO
        await rbt.set_lights_on_rgb(255, 0, 0)
        await rbt.set_wheel_speeds(0, 0)

        # ── 2. Registro de lugar (si es nuevo) y creación de arista con el anterior ──
        # Si la pose actual no coincide (en radio POS_EPS) con un lugar ya registrado,
        # se genera un nuevo `Place` y se persiste línea-a-línea.
        pos = await rbt.get_position()
        print(f"   Posición actual: ({pos.x:.1f}, {pos.y:.1f}) θ={pos.heading:.1f}°")
        print(f"   Sensores IR: Front={ir[3]}, Left={ir_left(ir)}, Right={ir_right(ir)}")

        current_place: Optional[Place] = map_manager.find_near_place(pos.x, pos.y)
        saved_new_place = False

        if current_place is None:
            current_place = Place(
                id=step_idx,
                x=pos.x, y=pos.y, theta=pos.heading,
                ir_front=ir[3], ir_left=ir_left(ir), ir_right=ir_right(ir),
                timestamp=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            )
            map_manager.append(current_place)
            print(f"   NUEVO LUGAR REGISTRADO: ID={current_place.id}")
            step_idx += 1
            saved_new_place = True
        else:
            print(f"   Posición ya visitada -> referencia a ID={current_place.id}")

        # Si existe un lugar previo y la decisión previa, registramos la arista (tramo).
        # La longitud del tramo se calcula por distancia Euclídea entre ambos lugares.
        if last_place is not None and last_turn is not None and current_place.id != last_place.id:
            segment_cm = last_place.distance_to(current_place.x, current_place.y)
            edge = Edge(
                from_id=last_place.id,
                to_id=current_place.id,
                turn=last_turn,
                segment_cm=segment_cm,
                start_x=last_place.x,
                start_y=last_place.y,
                end_x=current_place.x,
                end_y=current_place.y,
                timestamp=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            )
            edges_manager.append(edge)
            print(f"   ARISTA REGISTRADA: {edge.from_id} --{edge.turn}/{edge.segment_cm:.1f}cm--> {edge.to_id}")

        # Actualiza último lugar para el siguiente tramo
        last_place = current_place

        # ── 3. ¿Fin de exploración? ───────────────────────────────────────
        # Si ambos laterales superan el umbral, se considera callejón sin salida.
        left_blocked = ir_left(ir) > IR_DIR_THRESHOLD
        right_blocked = ir_right(ir) > IR_DIR_THRESHOLD
        print(f"   Bloqueos: Izquierda={'SÍ' if left_blocked else 'NO'}, Derecha={'SÍ' if right_blocked else 'NO'}")
        
        if left_blocked and right_blocked:
            print("   AMBAS RUTAS BLOQUEADAS -> FINALIZAR EXPLORACIÓN")
            # Final: LUZ VERDE + sonido de cierre
            await rbt.set_lights_on_rgb(0, 255, 0)
            await rbt.play_note(523, 0.6)
            break  # sin salidas posibles

        # ── 4. Decidir giro ───────────────────────────────────────────────
        # Heurística simple: elegir el lado con menor obstrucción relativa.
        # Se registra `last_turn` para poder crear la arista cuando se alcance
        # el siguiente lugar en el bucle.
        # Inspección/decisión: AMARILLO
        await rbt.set_lights_on_rgb(255, 255, 0)
        print("   ANÁLISIS DE DECISIÓN:")
        if ir_left(ir) < ir_right(ir):
            print("   Izquierda menos obstruida -> GIRAR IZQUIERDA")
            await rbt.turn_left(90)
            last_turn = 'left'
        else:
            print("   Derecha menos obstruida -> GIRAR DERECHA")
            await rbt.turn_right(90)
            last_turn = 'right'
        # Tras decidir y girar, volver a color de avance (azul)
        await rbt.set_lights_on_rgb(0, 0, 255)

        # Sonido breve si en este ciclo registramos un nuevo lugar y además giramos
        if saved_new_place:
            await rbt.play_note(880, 0.2)

    # ── Resumen final ──────────────────────────────────────────────────────
    print(f"\nEXPLORACIÓN FINALIZADA")
    print(f"   Total de pasos ejecutados: {step_count}")
    print(f"   Lugares registrados: {len(map_manager.places)}")
    print(f"   Archivo guardado: {FILE_NAME.resolve()}")
    print(f"   Aristas registradas: {len(edges_manager.edges)}")
    print(f"   Archivo de aristas: {EDGES_FILE_NAME.resolve()}")
    print("\nLugares registrados:")
    for i, p in enumerate(map_manager.places):
        print(f"   Lugar {i}: ({p.x:.1f}, {p.y:.1f}) θ={p.theta:.1f}° IR[L={p.ir_left},F={p.ir_front},R={p.ir_right}]")

# ── Lanza ejecución directa ───────────────────────────────────────────────
if __name__ == "__main__":
    map_manager = MapManager(FILE_NAME)  # instancia global independiente
    edges_manager = EdgeManager(EDGES_FILE_NAME)
    robot.play()
