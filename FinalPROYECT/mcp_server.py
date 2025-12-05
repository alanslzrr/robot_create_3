"""
Servidor MCP para Robot Create3 - Navegación Topológica
Usando FastMCP con transporte SSE (Server-Sent Events)

IMPORTANTE: 
- El robot se conecta durante la inicialización en el HILO PRINCIPAL
- Todas las herramientas son SÍNCRONAS para evitar conflictos con anyio
- La navegación se ejecuta en un hilo separado

Autores:
    - Alan Ariel Salazar
    - Yago Ramos Sánchez

Institución:
    Universidad Intercontinental de la Empresa (UIE)

Profesor:
    Eladio Dapena

Ejecutar con:
    python mcp_server.py

El servidor estará disponible en:
    - SSE endpoint: http://localhost:8001/sse
    - Messages endpoint: http://localhost:8001/messages
"""

import sys
import json
import logging
import os
from typing import Optional

# Configurar logging antes de todo
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler(sys.stderr)]
)
logger = logging.getLogger("mcp_server")

# ============================================================
# IMPORTS
# ============================================================
from mcp.server.fastmcp import FastMCP
from robot_service import RobotController

# ============================================================
# CONFIGURACIÓN
# ============================================================
MCP_PORT = 8001
MCP_HOST = "0.0.0.0"

# Variable de entorno para deshabilitar conexión de robot (testing)
CONNECT_ROBOT = os.getenv("CONNECT_ROBOT", "true").lower() != "false"

# ============================================================
# CONTROLADOR GLOBAL
# ============================================================
controller: Optional[RobotController] = None

def initialize_controller():
    """
    Inicializa el controlador del robot.
    DEBE ejecutarse en el hilo principal para que signal handlers funcionen.
    """
    global controller
    if controller is None:
        logger.info("=" * 50)
        logger.info("INICIALIZANDO CONTROLADOR DEL ROBOT")
        logger.info("=" * 50)
        
        if CONNECT_ROBOT:
            logger.info("Modo: ROBOT FÍSICO (Bluetooth)")
        else:
            logger.info("Modo: SIMULACIÓN (sin robot)")
        
        controller = RobotController(connect_robot=CONNECT_ROBOT)
        
        if controller.robot_ready:
            logger.info("✅ Controlador listo con robot conectado")
        else:
            logger.warning("⚠️ Controlador listo sin robot (modo simulación)")
        
        logger.info("=" * 50)
    return controller

def get_controller() -> RobotController:
    """Obtiene el controlador (debe estar ya inicializado)"""
    global controller
    if controller is None:
        raise RuntimeError("Controlador no inicializado. Llama initialize_controller() primero.")
    return controller

# ============================================================
# SERVIDOR MCP
# ============================================================
mcp = FastMCP(
    "Create3 Navigator",
    host=MCP_HOST,
    port=MCP_PORT
)

# ============================================================
# HERRAMIENTAS (TODAS SÍNCRONAS)
# ============================================================

@mcp.tool()
def list_available_locations() -> str:
    """
    Lista todos los nodos/ubicaciones disponibles en el mapa topológico del robot.
    Retorna un diccionario con los IDs de nodos y sus nombres descriptivos.
    """
    logger.info("Tool llamada: list_available_locations")
    try:
        ctrl = get_controller()
        nodes = ctrl.get_node_names()
        result = json.dumps(nodes, indent=2, ensure_ascii=False)
        logger.info(f"Nodos disponibles: {len(nodes)}")
        return result
    except Exception as e:
        logger.error(f"Error en list_available_locations: {e}")
        return json.dumps({"error": str(e)})


@mcp.tool()
def navigate_robot(destination_id: int, origin_id: int) -> str:
    """
    Navega el robot desde un nodo origen hacia un nodo destino.
    La navegación se ejecuta en background y retorna inmediatamente.
    
    IMPORTANTE: SIEMPRE debes proporcionar AMBOS parámetros.
    Si no conoces el origin_id, llama PRIMERO a get_robot_status() para obtener current_node.
    
    Args:
        destination_id: ID del nodo destino (REQUERIDO)
        origin_id: ID del nodo origen (REQUERIDO - usa current_node de get_robot_status si no lo sabes)
    
    Returns:
        Mensaje de confirmación o error
    """
    logger.info(f"Tool llamada: navigate_robot(dest={destination_id}, origin={origin_id})")
    try:
        ctrl = get_controller()
        result = ctrl.start_navigation(destination_id, origin_id)
        logger.info(f"navigate_robot resultado: {result}")
        return result
    except Exception as e:
        error_msg = f"Error en navigate_robot: {str(e)}"
        logger.error(error_msg)
        return json.dumps({"error": error_msg})


@mcp.tool()
def emergency_stop() -> str:
    """
    Detiene el robot inmediatamente y cancela cualquier misión en curso.
    Usar en caso de emergencia o cuando el usuario dice "para" o "detente".
    """
    logger.info("Tool llamada: emergency_stop")
    try:
        ctrl = get_controller()
        result = ctrl.emergency_stop()
        logger.info(f"emergency_stop resultado: {result}")
        return result
    except Exception as e:
        error_msg = f"Error en emergency_stop: {str(e)}"
        logger.error(error_msg)
        return json.dumps({"error": error_msg})


@mcp.tool()
def get_robot_status() -> str:
    """
    Obtiene el estado actual del robot. LLAMA A ESTA FUNCIÓN ANTES DE NAVEGAR
    si no conoces la posición actual del robot.
    
    Retorna:
    - state: IDLE, NAVIGATING, ERROR
    - current_node: ID del nodo donde está el robot actualmente (USAR COMO origin_id para navigate_robot)
    - target_node: nodo destino si está navegando
    - robot_connected: true/false
    - position: {x, y, theta}
    - last_heading: último ángulo conocido
    - last_message: último mensaje del sistema
    
    IMPORTANTE: El campo 'current_node' es el que debes usar como origin_id en navigate_robot()
    cuando el usuario dice "donde está", "el último", "su posición actual", etc.
    """
    logger.info("Tool llamada: get_robot_status")
    try:
        ctrl = get_controller()
        status = ctrl.get_status()
        result = json.dumps(status, indent=2, ensure_ascii=False)
        logger.info(f"Estado actual: {status.get('state', 'UNKNOWN')}, current_node: {status.get('current_node')}")
        return result
    except Exception as e:
        error_msg = f"Error en get_robot_status: {str(e)}"
        logger.error(error_msg)
        return json.dumps({"error": error_msg})


# ============================================================
# PUNTO DE ENTRADA
# ============================================================
if __name__ == "__main__":
    print("=" * 60, file=sys.stderr)
    print("SERVIDOR MCP - ROBOT CREATE3 NAVIGATOR", file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    print(f"Host: {MCP_HOST}", file=sys.stderr)
    print(f"Puerto: {MCP_PORT}", file=sys.stderr)
    print(f"SSE Endpoint: http://{MCP_HOST}:{MCP_PORT}/sse", file=sys.stderr)
    print(f"Messages Endpoint: http://{MCP_HOST}:{MCP_PORT}/messages", file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    
    # Esto permite que signal.signal() funcione correctamente
    try:
        initialize_controller()
    except Exception as e:
        logger.error(f"Error fatal inicializando controlador: {e}")
        logger.error("El servidor NO iniciará")
        sys.exit(1)
    
    print("\nIniciando servidor MCP con transporte SSE...", file=sys.stderr)
    
    # Iniciar servidor con transporte SSE
    mcp.run(transport="sse")