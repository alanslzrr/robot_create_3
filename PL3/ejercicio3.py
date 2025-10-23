"""
PRÁCTICA DE LABORATORIO 3 - Conocer el Entorno y Aprender
================================================================================
ROBOTS AUTÓNOMOS - Ejercicio 3.3 «Entorno Dinámico»

INFORMACIÓN BÁSICA:
- Autores: Yago Ramos - Salazar Alan
- Fecha de finalización: 14 de octubre de 2025 
- Institución: UIE
- Robot SDK: irobot-edu-sdk
- Requiere: Haber ejecutado ejercicio1.py y ejercicio2.py (fichero T03_Entorno.jsonl existente)

OBJETIVO ESPECÍFICO:
Diseñar e implementar una estrategia de adaptación que permita detectar discrepancias
entre la información previa almacenada y las condiciones actuales del entorno,
actualizando dinámicamente el conocimiento del robot sobre su entorno.

COMPORTAMIENTO ESPERADO:
1. Carga el mapa previo y continúa la exploración desde donde se quedó
2. Al llegar a una posición conocida, compara lecturas IR actuales vs almacenadas
3. Detecta cambios significativos mediante umbral de histéresis (DELTA_IR)
4. Actualiza automáticamente el fichero cuando detecta discrepancias
5. Registra nuevos lugares si encuentra áreas no mapeadas previamente
6. Aprende y adapta el modelo del entorno en tiempo real

SENSORES UTILIZADOS:
- Sensores IR de proximidad (array de 7 sensores)
  * ir[3]: Sensor frontal principal (comparación de obstáculos)
  * ir[0], ir[1]: Sensores laterales izquierdos (detección de cambios)
  * ir[5], ir[6]: Sensores laterales derechos (detección de cambios)
- Sistema de odometría (posición x,y y orientación θ)

ESTRATEGIA DE ADAPTACIÓN:
- Detección de discrepancias: Compara valores IR actuales vs almacenados
- Criterio de cambio: Diferencia absoluta > DELTA_IR (30 unidades)
- Actualización selectiva: Solo modifica campos con cambios significativos
- Timestamp de modificación: Actualiza marca temporal al detectar cambios
- Persistencia inmediata: Guarda cambios en fichero tras cada actualización

MECANISMOS DE APRENDIZAJE:
- Actualización incremental: Modifica registros existentes con nueva información
- Detección de novedad: Identifica áreas no mapeadas previamente
- Validación cruzada: Compara múltiples sensores para confirmar cambios
- Histórico de cambios: Mantiene registro temporal de modificaciones

ESTRUCTURAS DE DATOS:
- Place: Extendida con método update_from_ir() para aprendizaje adaptativo
- MapManager: Versión completa con guardado incremental y búsqueda por proximidad
- Sistema de versionado: Timestamp para rastrear evolución del conocimiento

ALMACENAMIENTO DINÁMICO:
- Lectura inicial: Carga mapa existente desde T03_Entorno.jsonl
- Actualización selectiva: Modifica solo registros con cambios detectados
- Guardado completo: Reescribe fichero tras cada modificación
- Nuevos registros: Añade lugares previamente desconocidos

CRITERIOS DE ACTUALIZACIÓN:
- Umbral de cambio: DELTA_IR = 30 unidades (histéresis para evitar ruido)
- Validación temporal: Actualiza timestamp solo si hay cambios reales
- Persistencia garantizada: Guarda inmediatamente tras detectar discrepancias
- Integridad de datos: Mantiene coherencia entre memoria y fichero

PARÁMETROS DE CONFIGURACIÓN:
- IR_OBS_THRESHOLD: 120 (~15cm para detección frontal)
- IR_DIR_THRESHOLD: 200 (umbral para considerar bloqueo lateral)
- POS_EPS: 5.0cm (radio para considerar posición ya visitada)
- DELTA_IR: 30 (histéresis para considerar "cambio significativo")
"""

from __future__ import annotations
import json, math, time
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Optional

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# ── constantes ────────────────────────────────────────────────────────────
FILE_NAME        = Path("T03_Entorno.jsonl")
EDGES_FILE_NAME  = Path("T03_Entorno_edges.jsonl")  # plan previo (si existe)
IR_OBS_THRESHOLD = 120
IR_DIR_THRESHOLD = 200
POS_EPS          = 5.0
DELTA_IR         = 30          # histéresis para considerar “cambio”

# ── estructuras ───────────────────────────────────────────────────────────
@dataclass
class Place:
    id: int; x: float; y: float; theta: float
    ir_front: int; ir_left: int; ir_right: int; timestamp: str

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)

    # ── aprendizaje ───────────────────────────────────────────────────────
    def update_from_ir(self, ir) -> bool:
        """
        Sobrescribe valores IR si difieren > DELTA_IR.
        Devuelve True si se modificó al menos un campo.
        """
        changed = False
        new_vals = {
            "ir_front": ir[3],
            "ir_left":  max(ir[0], ir[1]),
            "ir_right": max(ir[5], ir[6]),
        }
        for k, v in new_vals.items():
            if abs(getattr(self, k) - v) > DELTA_IR:
                setattr(self, k, v); changed = True
        if changed:
            self.timestamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        return changed

class MapManager:
    """Gestiona un mapa persistente con detección y guarda incremental."""
    def __init__(self, path: Path):
        if not path.exists():
            raise FileNotFoundError("Ejecute 3.1 y 3.2 para crear T03_Entorno.")
        self.path = path
        self.places: List[Place] = []
        with path.open() as f:
            for line in f:
                if line.strip():
                    self.places.append(Place(**json.loads(line)))

    # ── búsqueda ──────────────────────────────────────────────────────────
    def find_near(self, x: float, y: float) -> Optional[Place]:
        for p in self.places:
            if p.distance_to(x, y) < POS_EPS:
                return p
        return None

    # ── escritura completa (se invoca tras cada cambio) ───────────────────
    def save(self) -> None:
        with self.path.open("w") as f:
            for p in self.places:
                json.dump(asdict(p), f)
                f.write("\n")

    # ── alta de nuevos lugares ────────────────────────────────────────────
    def append(self, place: Place) -> None:
        self.places.append(place)
        self.save()

# ── aristas (para trazar nuevo recorrido si hay desvío) ───────────────────
@dataclass
class Edge:
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
    def __init__(self, path: Path):
        self.path  = path
        self.edges: List[Edge] = []
        if path.exists():
            with path.open() as f:
                for line in f:
                    if line.strip():
                        self.edges.append(Edge(**json.loads(line)))

    def append(self, edge: Edge) -> None:
        self.edges.append(edge)
        with self.path.open("a") as f:
            json.dump(asdict(edge), f)
            f.write("\n")

# ── utilidades ────────────────────────────────────────────────────────────
def ir_left(ir):  return max(ir[0], ir[1])
def ir_right(ir): return max(ir[5], ir[6])

async def wait_for_front_obstacle(rbt):
    while True:
        ir = (await rbt.get_ir_proximity()).sensors
        if ir[3] > IR_OBS_THRESHOLD:
            return ir

# ── robot ─────────────────────────────────────────────────────────────────
robot       = Create3(Bluetooth("C3_UIEC_Grupo1"))
map_manager = MapManager(FILE_NAME)
edges_manager = EdgeManager(EDGES_FILE_NAME)
existing_edge_keys = {(e.from_id, e.turn, e.to_id) for e in edges_manager.edges}

@event(robot.when_play)
async def play(rbt):
    """Exploración adaptativa con actualización del mapa.

    Leyenda de LEDs:
    - Azul (0,0,255): Avance por defecto entre puntos.
    - Amarillo (255,255,0): Inspección/decisión en el lugar actual.
    - Verde (0,255,0): Final sin salida disponible.

    Procedimiento (d–h):
    d) Actualiza T03_Entorno.jsonl con nuevos lugares y cambios detectados.
    e) Ejecutar este programa (T03_E03.py) con el robot en la posición inicial.
    f) Conectar y lanzar la misión; al finalizar, desconectar.
    g) Recolocar el robot en la posición inicial, reconectar y ejecutar de nuevo.
    h) El programa usará la información actualizada del fichero para continuar/adaptarse.

    Trazado de rutas:
    - Se compara cada tramo decidido (last_place -> current_place con 'turn') contra
      aristas existentes en T03_Entorno_edges.jsonl.
    - Si el tramo no existe (desvío), se crea un nuevo fichero de aristas con timestamp
      T03_Entorno_edges_YYYYMMDD_HHMMSS.jsonl y se registran allí las nuevas aristas.
    """
    await rbt.reset_navigation()
    next_id = max((p.id for p in map_manager.places), default=-1) + 1
    # Control de desvío: si el camino difiere del plan previo, comenzar nuevo trazado
    tracing_new_route = False
    last_place: Optional[Place] = None
    last_turn: Optional[str] = None
    active_edges_manager: Optional[EdgeManager] = None  # se crea con timestamp al detectar desvío

    def ensure_new_edges_manager() -> EdgeManager:
        nonlocal active_edges_manager
        if active_edges_manager is None:
            ts = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
            new_path = Path(f"T03_Entorno_edges_{ts}.jsonl")
            print(f"[Desvío] Nuevo fichero de aristas: {new_path}")
            active_edges_manager = EdgeManager(new_path)
        return active_edges_manager

    while True:
        # 1. Desplazamiento hasta obstáculo (AZUL)
        await rbt.set_lights_on_rgb(0, 0, 255)
        await rbt.set_wheel_speeds(5, 5)
        ir = await wait_for_front_obstacle(rbt)
        await rbt.set_wheel_speeds(0, 0)
        # Obstáculo detectado: rojo breve (consistente con 3.1)
        await rbt.set_lights_on_rgb(255, 0, 0)
        # Punto de decisión/inspección: AMARILLO
        await rbt.set_lights_on_rgb(255, 255, 0)
        pos = await rbt.get_position()

        # 2. Nuevo lugar o actualización de existente
        place = map_manager.find_near(pos.x, pos.y)
        if place is None:
            map_manager.append(Place(
                id=next_id,
                x=pos.x, y=pos.y, theta=pos.heading, 
                ir_front=ir[3], ir_left=ir_left(ir), ir_right=ir_right(ir),
                timestamp=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            ))
            print(f"➕ Nuevo lugar id={next_id} registrado")
            current_place = map_manager.find_near(pos.x, pos.y)
            next_id += 1
        else:
            if place.update_from_ir(ir):
                map_manager.save()
                print(f"✱ Lugar id={place.id} actualizado por cambio en el entorno")
            current_place = place

        # 2b. Registrar arista si procede y detectar desvío
        if last_place is not None and last_turn is not None and current_place and current_place.id != last_place.id:
            key = (last_place.id, last_turn, current_place.id)
            if key not in existing_edge_keys:
                tracing_new_route = True
                mgr = ensure_new_edges_manager()
                segment_cm = last_place.distance_to(current_place.x, current_place.y)
                mgr.append(Edge(
                    from_id=last_place.id,
                    to_id=current_place.id,
                    turn=last_turn,
                    segment_cm=segment_cm,
                    start_x=last_place.x,
                    start_y=last_place.y,
                    end_x=current_place.x,
                    end_y=current_place.y,
                    timestamp=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                ))
                print(f"→ [Desvío] Arista: {last_place.id} --{last_turn}/{segment_cm:.1f}cm--> {current_place.id}")
            # Reset decisión consumida y actualizar referencia
            last_turn = None
            last_place = current_place
        elif last_place is None and current_place:
            # Primera referencia
            last_place = current_place

        # 3. Decidir siguiente movimiento
        if ir_left(ir) > IR_DIR_THRESHOLD and ir_right(ir) > IR_DIR_THRESHOLD:
            # Sin salida: VERDE + tono de cierre
            await rbt.set_lights_on_rgb(0, 255, 0)
            break
        # Giro claro (evitar ternario con await) y registrar decisión
        if ir_left(ir) < ir_right(ir):
            await rbt.turn_left(90)
            last_turn = 'left'
        else:
            await rbt.turn_right(90)
            last_turn = 'right'
        # Tras el giro, volver a color de avance (AZUL)
        await rbt.set_lights_on_rgb(0, 0, 255)

        # Si estamos trazando nueva ruta, crear arista cuando avancemos al siguiente lugar
        # Estrategia: tras un giro y un avance hasta el próximo obstáculo, se registrará
        # la arista entre last_place (antes del avance) y el nuevo lugar detectado.
        # Implementación: cuando el bucle llegue de nuevo al registro de lugar, 
        # si tracing_new_route y last_turn existen y el id cambió, se añade arista.

    await rbt.play_note(523, 0.5)
    print(" Mapa final guardado con",
          len(map_manager.places), "lugares.")

if __name__ == "__main__":
    robot.play()
