"""
Módulo de seguridad para navegación autónoma del iRobot Create 3

Autores: Alan Salazar, Yago Ramos
Fecha: 4 de noviembre de 2025
Institución: UIE Universidad Intercontinental de la Empresa
Asignatura: Robots Autónomos - Profesor Eladio Dapena
Robot SDK: irobot-edu-sdk

OBJETIVOS PRINCIPALES:

En este módulo implementamos todas las funciones relacionadas con la seguridad
del robot durante la navegación autónoma. Nuestro objetivo principal era crear
una capa de protección robusta que garantizara operación segura del robot y
prevención de colisiones mediante múltiples mecanismos de detección y control.

Los objetivos específicos que buscamos alcanzar incluyen:

1. Implementar saturación de velocidades que limite los comandos a rangos seguros
   del hardware del robot, previniendo daños a los motores
2. Desarrollar un sistema de detección temprana de obstáculos usando sensores IR
   que permita reducir velocidad antes de que ocurran colisiones físicas
3. Implementar manejo de colisiones físicas mediante bumpers para detectar cuando
   el robot ha chocado y necesitamos detener el movimiento
4. Proporcionar funciones de reducción progresiva de velocidad según la proximidad
   de obstáculos detectados, implementando un sistema de factores de reducción
5. Integrar todas estas funciones como capas independientes que trabajan en conjunto
   para proporcionar múltiples niveles de protección

CONFIGURACIÓN:

Este módulo utiliza los umbrales y parámetros definidos en config.py. El sistema
de seguridad está basado en calibración real de los sensores IR con obstáculos
a diferentes distancias. Los umbrales están organizados en un sistema escalonado
que permite reacción gradual según la gravedad de la situación detectada.
"""

from . import config


# ============ FUNCIONES DE SATURACIÓN ============

def saturate_wheel_speeds(v_left, v_right):
    """
    Satura las velocidades de las ruedas para mantenerlas en rangos seguros.
    
    Esta función es esencial para proteger los motores del robot de comandos
    que excedan las capacidades físicas del hardware. Implementamos una doble
    estrategia: primero escalamos proporcionalmente si alguna rueda excede el
    máximo (manteniendo la relación entre ruedas), y luego saturaremos cada
    rueda individualmente para garantizar que nunca excedan los límites.
    
    El escalado proporcional es importante porque mantiene la intención del
    control (por ejemplo, si queremos girar, mantenemos la diferencia entre
    ruedas), mientras que la saturación individual proporciona una capa adicional
    de seguridad por si el cálculo anterior produjo valores fuera de rango.
    
    Args:
        v_left: Velocidad deseada para la rueda izquierda en cm/s
        v_right: Velocidad deseada para la rueda derecha en cm/s
    
    Returns:
        tuple: Tupla (v_left_sat, v_right_sat) con las velocidades saturadas
               dentro de los límites seguros del robot
    """
    # Encontrar el máximo absoluto entre ambas ruedas
    # Esto nos permite determinar si necesitamos escalar proporcionalmente
    max_abs = max(abs(v_left), abs(v_right))
    
    # Si alguna rueda excede el máximo, escalar ambas proporcionalmente
    # Esto mantiene la relación entre las velocidades (importante para giros)
    # mientras las lleva dentro del rango seguro
    if max_abs > config.V_MAX_CM_S:
        scale = config.V_MAX_CM_S / max_abs
        v_left *= scale
        v_right *= scale
    
    # Saturación individual de cada rueda como capa adicional de seguridad
    # Esto garantiza que incluso si el escalado anterior no fue suficiente,
    # nunca excederemos los límites físicos del robot
    v_left = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_left))
    v_right = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_right))
    
    return v_left, v_right


# ============ FUNCIONES DE DETECCIÓN DE OBSTÁCULOS ============

def detect_obstacle(ir_sensors):
    """
    Detecta obstáculos mediante análisis de sensores IR frontales.
    
    Esta función analiza las lecturas de los sensores IR frontales más críticos
    (sensores 1, 2, 3, 4 según nuestra calibración) y determina el nivel de
    peligro basándose en umbrales calibrados experimentalmente. Retorna información
    sobre si debemos detener, reducir velocidad, y un factor de reducción proporcional.
    
    El sistema utiliza umbrales binarios para compatibilidad con código que espera
    esta estructura, pero está basado en el sistema de umbrales escalonados más
    avanzado implementado en potential_fields.py.
    
    Args:
        ir_sensors: Lista con 7 valores de sensores IR (índices 0-6) en rango 0-4095
    
    Returns:
        dict: Diccionario con información de detección que contiene:
            - stop: bool, True si debe detenerse completamente (IR > umbral crítico)
            - slow: bool, True si debe reducir velocidad (IR > umbral de advertencia)
            - max_front: int, valor máximo detectado en sensores frontales críticos
            - speed_factor: float (0.0-1.0), factor de reducción de velocidad proporcional
    """
    # Validar que tengamos lecturas de sensores válidas
    if not ir_sensors or len(ir_sensors) < 7:
        return {'stop': False, 'slow': False, 'max_front': 0, 'speed_factor': 1.0}
    
    # Analizar los sensores frontales más críticos según nuestra calibración
    # Estos sensores detectan obstáculos directamente en la trayectoria del robot
    front_vals = [
        ir_sensors[config.IR_SIDE_LEFT],     # Sensor 1: lateral izquierdo
        ir_sensors[config.IR_FRONT_LEFT],    # Sensor 2: frontal izquierdo
        ir_sensors[config.IR_FRONT_CENTER],  # Sensor 3: central (más sensible)
        ir_sensors[config.IR_FRONT_RIGHT]    # Sensor 4: frontal derecho
    ]
    max_front = max(front_vals)
    
    # Determinar niveles de peligro usando umbrales calibrados
    # Estos umbrales están basados en calibración real con obstáculos a 5cm
    stop = max_front > config.IR_THRESHOLD_STOP
    slow = max_front > config.IR_THRESHOLD_SLOW
    
    # Calcular factor de reducción de velocidad proporcional
    # Este factor varía suavemente entre 1.0 (sin obstáculos) y 0.0 (obstáculo muy cerca)
    if max_front > config.IR_THRESHOLD_STOP:
        # Situación crítica: detener completamente
        speed_factor = 0.0
    elif max_front > config.IR_THRESHOLD_SLOW:
        # Situación de advertencia: reducir velocidad proporcionalmente
        # El factor disminuye linealmente entre los umbrales de advertencia y crítico
        speed_factor = max(0.3, 1.0 - (max_front - config.IR_THRESHOLD_SLOW) / 
                          (config.IR_THRESHOLD_STOP - config.IR_THRESHOLD_SLOW))
    else:
        # Sin obstáculos detectados: velocidad normal
        speed_factor = 1.0
    
    return {
        'stop': stop,
        'slow': slow,
        'max_front': max_front,
        'speed_factor': speed_factor
    }


# ============ FUNCIONES DE DETECCIÓN DE COLISIONES ============

def check_bumpers(bumpers):
    """
    Verifica el estado de los bumpers físicos para detectar colisiones.
    
    Esta función proporciona información detallada sobre el estado de los bumpers,
    indicando si hay colisión y de qué lado. Es útil para logging y manejo de
    errores que requieren información específica sobre la naturaleza de la colisión.
    
    Args:
        bumpers: Tupla (left, right) con estados booleanos de los bumpers
                 True indica que el bumper está presionado (colisión detectada)
    
    Returns:
        dict: Diccionario con información de colisión que contiene:
            - collision: bool, True si hay cualquier colisión
            - left: bool, True si hay colisión en el lado izquierdo
            - right: bool, True si hay colisión en el lado derecho
    """
    left, right = bumpers
    return {
        'collision': left or right,
        'left': left,
        'right': right
    }


def emergency_stop_needed(bumpers):
    """
    Determina si se necesita una parada de emergencia basada en bumpers físicos.
    
    Esta función es la interfaz principal para detectar colisiones físicas reales.
    A diferencia de la detección mediante sensores IR, los bumpers solo se activan
    cuando ya ha ocurrido contacto físico, por lo que indican una situación de
    emergencia que requiere detención inmediata.
    
    Utilizamos esta función en los bucles de navegación para detectar cuando el
    robot ha chocado físicamente contra un obstáculo, permitiendo implementar
    estrategias de recuperación como retroceso y reintento.
    
    Args:
        bumpers: Tupla (left, right) con estados booleanos de los bumpers
    
    Returns:
        bool: True si hay colisión física detectada por cualquiera de los bumpers
    """
    return bumpers[0] or bumpers[1]


# ============ FUNCIONES DE REDUCCIÓN DE VELOCIDAD ============

def apply_obstacle_slowdown(v_left, v_right, ir_sensors):
    """
    Aplica reducción de velocidad basada en detección de obstáculos mediante IR.
    
    Esta función integra la detección de obstáculos con la aplicación de correcciones
    de velocidad. Primero analiza los sensores IR para determinar el nivel de peligro,
    y luego aplica el factor de reducción correspondiente a las velocidades calculadas.
    
    Esta función se utiliza principalmente en PRM01_P01.py donde aplicamos reducción
    de velocidad como capa adicional de seguridad. En PRM01_P02.py, el potencial
    repulsivo ya maneja la evasión, por lo que esta función no se utiliza directamente
    en el bucle principal.
    
    Args:
        v_left: Velocidad calculada para la rueda izquierda en cm/s
        v_right: Velocidad calculada para la rueda derecha en cm/s
        ir_sensors: Lista de 7 valores de sensores IR (rango 0-4095)
    
    Returns:
        tuple: Tupla (v_left_reduced, v_right_reduced, obs_info) donde:
            - v_left_reduced: Velocidad izquierda reducida según detección
            - v_right_reduced: Velocidad derecha reducida según detección
            - obs_info: Diccionario con información de detección de obstáculos
    """
    # Detectar obstáculos y obtener información de peligro
    obs = detect_obstacle(ir_sensors)
    
    # Aplicar el factor de reducción a ambas velocidades
    # Este factor varía entre 0.0 (detener) y 1.0 (velocidad normal)
    factor = obs['speed_factor']
    
    return v_left * factor, v_right * factor, obs