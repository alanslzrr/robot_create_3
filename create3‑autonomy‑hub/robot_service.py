"""
Servicio de control del robot Create3 para integración MCP

Este módulo encapsula toda la lógica de navegación del robot Create3,
gestionando la conexión Bluetooth mediante un HILO DEDICADO para el SDK.

IMPORTANTE: 
- El robot se conecta en el HILO PRINCIPAL durante la inicialización
- La navegación se ejecuta en un hilo separado con su propio event loop
- Esto evita conflictos con anyio (FastMCP) y signal handlers

Autores:
    - Alan Ariel Salazar
    - Yago Ramos Sánchez

Institución:
    Universidad Intercontinental de la Empresa (UIE)

Profesor:
    Eladio Dapena

Asignatura:
    Robots Autónomos
"""

import asyncio
import json
import sys
import threading
import logging
import time
from pathlib import Path
from typing import Literal, Optional, Dict, Callable, Any

# Configurar logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler(sys.stderr)]
)
logger = logging.getLogger("robot_service")

# Importaciones de nuestro proyecto existente
from src import config
from grafos.prueba import cargar_grafo_desde_json
from PRM02_P02_EQUIPO1_grafos import CombinedPotentialNavigator

# Importaciones del SDK iRobot
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import Create3

STATE_FILE = "last_state.json"


class NavigationThread(threading.Thread):
    """
    Hilo dedicado para ejecutar la navegación del robot.
    Tiene su propio event loop de asyncio para evitar conflictos con anyio.
    NOTA: El robot ya debe estar conectado antes de iniciar este hilo.
    """
    
    def __init__(self, controller: 'RobotController', origin_node: int, dest_node: int):
        super().__init__(daemon=True)
        self.controller = controller
        self.origin_node = origin_node
        self.dest_node = dest_node
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self._stop_event = threading.Event()
        
    def run(self):
        """Ejecuta la navegación en un event loop propio"""
        logger.info(f"[NavThread] Iniciando hilo de navegación {self.origin_node} -> {self.dest_node}")
        
        # Crear nuevo event loop para este hilo
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        try:
            self.loop.run_until_complete(self._run_mission())
        except Exception as e:
            logger.error(f"[NavThread] Error en navegación: {e}")
            import traceback
            traceback.print_exc(file=sys.stderr)
            self.controller.state = "ERROR"
            self.controller.last_message = f"Error en navegación: {str(e)}"
        finally:
            logger.info("[NavThread] Hilo de navegación finalizado")
            self.loop.close()
            self.controller.navigation_thread = None
            self.controller.target_node = None
    
    def stop(self):
        """Señala al hilo que debe detenerse"""
        self._stop_event.set()
        
    async def _run_mission(self):
        """Ejecuta la misión de navegación"""
        try:
            # Verificar que el robot está conectado
            if not self.controller.robot_ready:
                raise ValueError("Robot no está conectado. Espera a que se complete la conexión Bluetooth.")
            
            if self._stop_event.is_set():
                logger.info("[NavThread] Navegación cancelada antes de iniciar")
                return
            
            # Cálculo de ruta
            logger.info(f"[NavThread] 📍 Planificando ruta {self.origin_node} -> {self.dest_node}...")
            path_indices, total_cost = self.controller.graph.Camino_Minimo_Dijkstra(
                self.origin_node, self.dest_node
            )
            
            if not path_indices:
                raise ValueError("Ruta imposible - no hay camino entre los nodos")
            
            logger.info(f"[NavThread] Ruta encontrada: {path_indices}, costo: {total_cost:.2f}")
            
            # Construcción de waypoints
            coords = self.controller.graph.coords
            q_i_node = coords[path_indices[0]]
            q_f_node = coords[path_indices[-1]]
            waypoint_indices = path_indices[1:-1]
            
            # IMPORTANTE: Para navegación consecutiva, necesitamos el heading REAL del robot,
            # no el theta almacenado en el grafo (que solo es válido para la posición inicial).
            # El robot puede haber llegado al nodo de origen desde cualquier dirección,
            # por lo que su heading actual puede ser diferente al theta del grafo.
            actual_theta = q_i_node["theta"]  # Valor por defecto del grafo
            
            try:
                # Intentar obtener el heading real del robot
                pos = await self.controller.robot.get_position()
                if pos is not None:
                    actual_theta = pos.heading
                    logger.info(f"[NavThread] Heading real del robot: {actual_theta:.1f}° (grafo: {q_i_node['theta']}°)")
                elif self.controller.last_heading is not None:
                    # Usar el heading guardado del último estado si no podemos leer del robot
                    actual_theta = self.controller.last_heading
                    logger.info(f"[NavThread] Usando heading guardado: {actual_theta:.1f}° (grafo: {q_i_node['theta']}°)")
            except Exception as e:
                # Si hay error, intentar usar el heading guardado
                if self.controller.last_heading is not None:
                    actual_theta = self.controller.last_heading
                    logger.warning(f"[NavThread] Error obteniendo heading, usando guardado: {actual_theta:.1f}° - {e}")
                else:
                    logger.warning(f"[NavThread] No se pudo obtener heading real, usando valor del grafo: {e}")
            
            q_i = (q_i_node["x"], q_i_node["y"], actual_theta)
            q_f = (q_f_node["x"], q_f_node["y"])
            waypoints = [(coords[i]["x"], coords[i]["y"]) for i in waypoint_indices]
            
            logger.info(f"[NavThread] Waypoints: {len(waypoints)}")
            
            # Instanciar Navegador
            # IMPORTANTE: Deshabilitamos logging para evitar conflictos de event loop
            # Los loggers usan asyncio.Lock que se vincula al event loop donde se crean
            self.controller.navigator_instance = CombinedPotentialNavigator(
                self.controller.robot, q_i, waypoints, q_f,
                potential_type='linear',
                k_rep=config.K_REPULSIVE,
                d_influence=config.D_INFLUENCE,
                debug=True,  # Habilitamos debug para ver qué pasa
                disable_logging=True  # Deshabilitamos logging para evitar conflictos de event loop
            )
            
            self.controller.mission_completed = False
            
            # EJECUTAR NAVEGACIÓN
            logger.info("[NavThread] 🤖 Ejecutando control de navegación...")
            success = await self.controller.navigator_instance.navigate()
            
            self.controller.mission_completed = True
            
            # Guardar el heading final del robot (importante para navegación consecutiva)
            try:
                final_pos = await self.controller.robot.get_position()
                if final_pos is not None:
                    self.controller.last_heading = final_pos.heading
                    logger.info(f"[NavThread] Heading final guardado: {final_pos.heading:.1f}°")
            except Exception as e:
                logger.warning(f"[NavThread] No se pudo obtener heading final: {e}")
            
            if success:
                self.controller.current_node = self.dest_node
                self.controller.state = "IDLE"
                self.controller.last_message = f"✅ Llegada exitosa al Nodo {self.dest_node}."
                self.controller._write_state_to_disk()
                logger.info(f"[NavThread] ✅ Navegación completada exitosamente")
            else:
                self.controller.state = "ERROR"
                self.controller.last_message = "❌ Navegación fallida (obstáculo o timeout)."
                # También guardamos el estado en caso de fallo para mantener el heading
                self.controller._write_state_to_disk()
                logger.warning("[NavThread] ❌ Navegación fallida")

        except asyncio.CancelledError:
            logger.warning("[NavThread] ⚠️ Navegación cancelada")
            self.controller.state = "IDLE"
            self.controller.last_message = "Navegación cancelada."
        except Exception as e:
            self.controller.state = "ERROR"
            self.controller.last_message = f"Error: {str(e)}"
            logger.error(f"[NavThread] ❌ Error en misión: {e}")
            import traceback
            traceback.print_exc(file=sys.stderr)
        finally:
            self.controller.target_node = None
            self.controller.navigator_instance = None


class RobotController:
    """
    Controlador central del robot Create3.
    
    IMPORTANTE: El robot se conecta en el hilo principal durante __init__
    para evitar problemas con signal handlers. La navegación se ejecuta
    en un hilo separado.
    """
    
    def __init__(self, connect_robot: bool = True):
        """
        Inicializa el controlador.
        
        Args:
            connect_robot: Si True, intenta conectar al robot Bluetooth.
                          Poner False para testing sin robot físico.
        """
        # 1. Carga del Grafo
        logger.info("Cargando grafo topológico...")
        try:
            self.graph = cargar_grafo_desde_json()
            logger.info(f"Grafo cargado: {self.graph.V} nodos")
        except Exception as e:
            logger.error(f"ERROR al cargar grafo: {e}")
            raise
        
        # 2. Configuración Robot
        self.robot_name = config.BLUETOOTH_NAME
        self.robot: Optional[Create3] = None
        self.robot_thread: Optional[threading.Thread] = None
        self.robot_ready: bool = False  # Flag para saber si está listo
        
        # 3. Estado Interno
        self.state: Literal["IDLE", "NAVIGATING", "ERROR"] = "IDLE"
        self.current_node: Optional[int] = None
        self.target_node: Optional[int] = None
        self.last_message: str = "Sistema inicializado."
        self.last_heading: Optional[float] = None  # Último heading conocido (para persistencia)
        
        # 4. Gestión de Navegación
        self.navigation_thread: Optional[NavigationThread] = None
        self.navigator_instance: Optional[CombinedPotentialNavigator] = None
        
        # 5. Telemetría
        self.telemetry_callback: Optional[Callable[[Dict[str, Any]], None]] = None
        self.current_position: Dict[str, float] = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.mission_completed: bool = False
        
        # 6. Persistencia
        self.load_state()
        
        # 7. Conectar robot en el hilo principal (CRÍTICO para signal handlers)
        if connect_robot:
            self._connect_robot_main_thread()
    
    def _connect_robot_main_thread(self):
        """
        Conecta al robot desde el hilo principal.
        Esto DEBE ejecutarse en el hilo principal para que signal.signal funcione.
        """
        logger.info(f"🔌 Conectando al robot {self.robot_name} desde hilo principal...")
        
        try:
            # Crear instancia de robot (esto registra signal handlers)
            self.robot = Create3(Bluetooth(self.robot_name))
            logger.info("Robot instanciado correctamente")
            
            # Iniciar hilo del SDK (robot.play) en daemon thread
            logger.info("🧵 Iniciando hilo del SDK iRobot...")
            self.robot_thread = threading.Thread(target=self.robot.play, daemon=True)
            self.robot_thread.start()
            
            # Esperar negociación Bluetooth con verificación activa
            logger.info("⏳ Esperando negociación Bluetooth...")
            max_wait = 15  # segundos máximo de espera
            check_interval = 1  # verificar cada segundo
            waited = 0
            connected = False
            
            while waited < max_wait:
                time.sleep(check_interval)
                waited += check_interval
                
                # Verificar conexión intentando obtener posición
                try:
                    # Crear un event loop temporal para la verificación
                    loop = asyncio.new_event_loop()
                    try:
                        pos = loop.run_until_complete(
                            asyncio.wait_for(self.robot.get_position(), timeout=2.0)
                        )
                        if pos is not None:
                            logger.info(f"✅ Robot conectado (pos: x={pos.x:.1f}, y={pos.y:.1f}) después de {waited}s")
                            connected = True
                            break
                    except asyncio.TimeoutError:
                        logger.info(f"   Esperando... ({waited}/{max_wait}s)")
                    except Exception as e:
                        # "Service Discovery" error es normal durante la conexión
                        if "Service Discovery" in str(e):
                            logger.info(f"   Descubriendo servicios... ({waited}/{max_wait}s)")
                        else:
                            logger.info(f"   Esperando... ({waited}/{max_wait}s) - {e}")
                    finally:
                        loop.close()
                except Exception as e:
                    logger.debug(f"   Error verificación: {e}")
            
            if connected:
                self.robot_ready = True
                logger.info("✅ Robot conectado y listo para navegación")
                self.last_message = "Robot conectado y listo."
            else:
                # Asumir que está conectando pero no verificado
                logger.warning("⚠️ No se pudo verificar conexión, pero el hilo está activo")
                logger.warning("   Intentando continuar - la navegación puede fallar si no está conectado")
                self.robot_ready = True  # Intentar de todos modos
                self.last_message = "Robot iniciado (conexión no verificada)"
            
        except Exception as e:
            logger.error(f"❌ Error conectando robot: {e}")
            logger.warning("El sistema funcionará sin robot físico (modo simulación)")
            self.robot_ready = False
            self.last_message = f"Robot no conectado: {str(e)}"
    
    def load_state(self):
        """Carga el estado persistente desde disco"""
        path = Path(STATE_FILE)
        if path.exists():
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self.current_node = data.get("current_node")
                    self.last_heading = data.get("last_heading")
                    if data.get("last_message"):
                        self.last_message = data.get("last_message")
                    logger.info(f"Estado cargado. Nodo actual: {self.current_node}, Heading: {self.last_heading}°")
            except Exception as e:
                logger.error(f"Error cargando estado: {e}")
        else:
            logger.info("No hay estado previo guardado")

    def _write_state_to_disk(self):
        """Guarda el estado actual a disco de forma síncrona"""
        data = {
            "current_node": self.current_node,
            "last_heading": self.last_heading,
            "last_message": self.last_message
        }
        try:
            with open(STATE_FILE, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
            logger.debug(f"Estado guardado: nodo={self.current_node}, heading={self.last_heading}°")
        except Exception as e:
            logger.error(f"Error guardando estado: {e}")
    
    def get_node_names(self) -> Dict[int, str]:
        """Retorna diccionario de ID -> nombre de nodo"""
        return {i: self.graph.nombres[i] for i in range(self.graph.V)}

    def emergency_stop(self) -> str:
        """
        Detiene inmediatamente el robot y cancela cualquier misión en curso.
        """
        msg = "Parada de emergencia ejecutada."
        logger.warning(f"🛑 {msg}")
        
        # Cancelar hilo de navegación
        if self.navigation_thread and self.navigation_thread.is_alive():
            self.navigation_thread.stop()
            msg += " Navegación cancelada."
            logger.info("Señal de cancelación enviada a navegación")
        
        # Intentar detener motores
        if self.robot and self.robot_ready:
            try:
                # Crear un event loop temporal solo para detener
                loop = asyncio.new_event_loop()
                try:
                    loop.run_until_complete(
                        asyncio.wait_for(self.robot.set_wheel_speeds(0, 0), timeout=2.0)
                    )
                    msg += " Motores detenidos."
                    logger.info("Motores detenidos")
                except Exception as e:
                    logger.error(f"Error deteniendo motores: {e}")
                finally:
                    loop.close()
            except Exception as e:
                logger.error(f"Error en emergency_stop: {e}")
        
        self.state = "IDLE"
        self.target_node = None
        self.last_message = msg
        self.navigator_instance = None
        
        self._write_state_to_disk()
        return msg
    
    def get_status(self) -> dict:
        """
        Obtiene el estado actual del robot.
        """
        return {
            "state": self.state,
            "current_node": self.current_node,
            "target_node": self.target_node,
            "robot_connected": self.robot_ready,
            "last_message": self.last_message,
            "last_heading": self.last_heading,
            "position": self.current_position.copy(),
            "mission_completed": self.mission_completed
        }
    
    def set_telemetry_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """Configura callback para telemetría"""
        self.telemetry_callback = callback

    def start_navigation(self, destination_id: int, origin_id: Optional[int] = None) -> str:
        """
        Inicia la navegación hacia un nodo destino.
        
        La navegación se ejecuta en un HILO SEPARADO con su propio event loop.
        
        Args:
            destination_id: ID del nodo destino
            origin_id: ID del nodo origen (opcional, usa el último conocido si no se especifica)
        
        Returns:
            Mensaje de confirmación o error
        """
        # Verificar conexión del robot
        if not self.robot_ready:
            msg = "ERROR: Robot no conectado. Reinicia el servidor para conectar."
            logger.warning(msg)
            return msg
        
        # Verificar si ya hay navegación en curso
        if self.state == "NAVIGATING" or (self.navigation_thread and self.navigation_thread.is_alive()):
            msg = f"ERROR: Robot ocupado navegando hacia nodo {self.target_node}."
            logger.warning(msg)
            return msg
        
        actual_origin = origin_id if origin_id is not None else self.current_node
        if actual_origin is None:
            msg = "ERROR: Origen desconocido. Indica origin_id."
            logger.warning(msg)
            return msg
        
        if not (0 <= actual_origin < self.graph.V and 0 <= destination_id < self.graph.V):
            msg = f"ERROR: IDs de nodo inválidos (origen={actual_origin}, destino={destination_id}, max={self.graph.V-1})."
            logger.warning(msg)
            return msg
        
        if actual_origin == destination_id:
            self.current_node = actual_origin
            self._write_state_to_disk()
            msg = f"INFO: Ya estás en el nodo {destination_id}."
            logger.info(msg)
            return msg
        
        # Actualizar estado
        self.state = "NAVIGATING"
        self.target_node = destination_id
        self.mission_completed = False
        self.last_message = f"Iniciando ruta: {actual_origin} -> {destination_id}."
        
        logger.info(f"🚀 Nueva misión: {actual_origin} -> {destination_id}")
        
        # Crear y arrancar hilo de navegación
        self.navigation_thread = NavigationThread(self, actual_origin, destination_id)
        self.navigation_thread.start()
        
        return f"RECIBIDO: Iniciando navegación desde Nodo {actual_origin} hacia Nodo {destination_id}."
