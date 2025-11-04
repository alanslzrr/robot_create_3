"""
Módulo de seguridad para navegación autónoma del iRobot Create 3

Autores: Yago Ramos - Salazar Alan
Fecha de finalización: 28 de octubre de 2025
Institución: UIE - Robots Autónomos
Robot SDK: irobot-edu-sdk

Objetivo:
    Proporcionar funciones de seguridad que protejan al robot y su entorno
    mediante saturación de velocidades, detección temprana de obstáculos con
    sensores infrarrojos y manejo de colisiones físicas con bumpers. Todas
    las funciones operan como capas de seguridad independientes del control
    de navegación principal.

Comportamiento esperado:
    - Limitar velocidades calculadas a rangos seguros del hardware
    - Detectar obstáculos antes del contacto físico usando sensores IR
    - Reducir velocidad proporcionalmente a la proximidad detectada
    - Detener el robot completamente ante colisión inminente (IR > 300)
    - Registrar colisiones físicas para conteo y manejo de errores

Funciones principales:
    
    saturate_wheel_speeds(v_left, v_right):
        Limita velocidades individuales y escala proporcionalmente si exceden
        el máximo permitido. Previene comandos que dañen los motores.
        
        Parámetros:
            v_left: Velocidad rueda izquierda en cm/s
            v_right: Velocidad rueda derecha en cm/s
        
        Retorna:
            (v_left_sat, v_right_sat): Velocidades saturadas
    
    detect_obstacle(ir_sensors):
        Analiza lecturas de sensores IR frontales (índices 1-4) y determina
        nivel de peligro con umbrales calibrados experimentalmente.
        
        Parámetros:
            ir_sensors: Lista con 7 valores de sensores IR (0-6)
        
        Retorna:
            dict con:
                - stop: bool, True si debe detenerse (IR > 300)
                - slow: bool, True si debe reducir velocidad (IR > 150)
                - max_front: int, valor máximo de sensores frontales
                - speed_factor: float (0.0-1.0), factor de reducción de velocidad
    
    emergency_stop_needed(bumpers):
        Verifica estado de bumpers físicos para detectar colisión real.
        
        Parámetros:
            bumpers: Tupla (left, right) con booleanos de contacto
        
        Retorna:
            bool: True si hay colisión física
    
    apply_obstacle_slowdown(v_left, v_right, ir_sensors):
        Aplica reducción de velocidad basada en detección IR de forma integrada.
        
        Parámetros:
            v_left, v_right: Velocidades calculadas (cm/s)
            ir_sensors: Lista de lecturas IR
        
        Retorna:
            (v_left_reduced, v_right_reduced, obs_info): Velocidades ajustadas
            y diccionario con información de detección

Umbrales de sensores IR (calibrados a 5 cm):
    - IR_THRESHOLD_DETECT: 50 (detección mínima)
    - IR_THRESHOLD_SLOW: 150 (inicio de reducción)
    - IR_THRESHOLD_STOP: 300 (parada completa)
"""

import config


def saturate_wheel_speeds(v_left, v_right):
    """
    Satura las velocidades de las ruedas para mantenerlas en rangos seguros.
    
    Args:
        v_left: Velocidad rueda izquierda (cm/s)
        v_right: Velocidad rueda derecha (cm/s)
    
    Returns:
        (v_left_sat, v_right_sat): Velocidades saturadas
    """
    # Encontrar el máximo absoluto
    max_abs = max(abs(v_left), abs(v_right))
    
    # Si excede el máximo, escalar proporcionalmente
    if max_abs > config.V_MAX_CM_S:
        scale = config.V_MAX_CM_S / max_abs
        v_left *= scale
        v_right *= scale
    
    # Saturar individualmente por seguridad
    v_left = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_left))
    v_right = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_right))
    
    return v_left, v_right


def detect_obstacle(ir_sensors):
    """
    Detecta obstáculos. Retorna nivel de peligro y velocidad sugerida.
    """
    if not ir_sensors or len(ir_sensors) < 7:
        return {'stop': False, 'slow': False, 'max_front': 0, 'speed_factor': 1.0}
    
    # Sensores frontales más críticos (1,2,3,4)
    front_vals = [
        ir_sensors[config.IR_SIDE_LEFT],     # 1
        ir_sensors[config.IR_FRONT_LEFT],    # 2
        ir_sensors[config.IR_FRONT_CENTER],  # 3
        ir_sensors[config.IR_FRONT_RIGHT]    # 4
    ]
    max_front = max(front_vals)
    
    stop = max_front > config.IR_THRESHOLD_STOP
    slow = max_front > config.IR_THRESHOLD_SLOW
    
    # Factor de velocidad según proximidad
    if max_front > config.IR_THRESHOLD_STOP:
        speed_factor = 0.0  # PARAR
    elif max_front > config.IR_THRESHOLD_SLOW:
        # Reducir proporcionalmente
        speed_factor = max(0.3, 1.0 - (max_front - config.IR_THRESHOLD_SLOW) / 
                          (config.IR_THRESHOLD_STOP - config.IR_THRESHOLD_SLOW))
    else:
        speed_factor = 1.0
    
    return {
        'stop': stop,
        'slow': slow,
        'max_front': max_front,
        'speed_factor': speed_factor
    }


def check_bumpers(bumpers):
    """
    Verifica si hay colisión física.
    
    Args:
        bumpers: Tupla (left, right) con estados de bumpers
    
    Returns:
        dict con:
            - 'collision': bool, True si hay colisión
            - 'left': bool, colisión izquierda
            - 'right': bool, colisión derecha
    """
    left, right = bumpers
    return {
        'collision': left or right,
        'left': left,
        'right': right
    }


def emergency_stop_needed(bumpers):
    """Parada SOLO si bumpers físicos activos (colisión real)."""
    return bumpers[0] or bumpers[1]


def apply_obstacle_slowdown(v_left, v_right, ir_sensors):
    """Reduce velocidades si hay obstáculo detectado."""
    obs = detect_obstacle(ir_sensors)
    factor = obs['speed_factor']
    return v_left * factor, v_right * factor, obs
