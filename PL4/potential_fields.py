"""
Implementación de funciones de potencial atractivo para navegación autónoma

Autores: Yago Ramos - Salazar Alan
Fecha de finalización: 28 de octubre de 2025
Institución: UIE - Robots Autónomos
Robot SDK: irobot-edu-sdk

Objetivo:
    Implementar cuatro variantes de campos de potencial atractivo que generen
    fuerzas proporcionales a la distancia hacia la meta, traducidas a velocidades
    diferenciales para las ruedas del robot. Cada función presenta características
    distintas de aceleración y convergencia.

Comportamiento esperado:
    - Calcular velocidades de rueda (v_left, v_right) basadas en posición actual
      y objetivo, sin realizar giros puros sobre el eje del robot
    - Combinar control lineal (avance hacia meta) y angular (corrección de orientación)
      en cada iteración del bucle de control
    - Aplicar rampa de aceleración progresiva desde arranque hasta velocidad máxima
    - Saturar velocidades dentro de límites físicos del robot
    - Mantener estabilidad mediante normalización de ángulos y factores de corrección

Funciones de potencial implementadas:
    1. Linear: F = k·d
       Fuerza proporcional a distancia. Comportamiento predecible y directo.
    
    2. Quadratic: F = k·d²/10
       Fuerza proporcional al cuadrado. Aceleración agresiva lejos, frenado suave cerca.
    
    3. Conic: F = k·min(d, 100)·2
       Saturación a 100cm. Velocidad constante lejos, decremento lineal cerca.
    
    4. Exponential: F = k·(1-e^(-d/50))·20
       Convergencia asintótica. Aceleración rápida, desaceleración muy suave.

Parámetros:
    - q: Tupla (x, y, theta_deg) con posición y orientación actual del robot
    - q_goal: Tupla (x_goal, y_goal) con coordenadas de la meta
    - k_lin: Ganancia lineal específica del tipo de potencial (auto-seleccionada)
    - k_ang: Ganancia angular para corrección de orientación (0.6 default)
    - potential_type: Tipo de función ['linear', 'quadratic', 'conic', 'exponential']

Variables globales:
    - _last_v_linear: Velocidad lineal de la iteración anterior para rampa de aceleración
"""
import math
import config

# Tipos de potencial disponibles
POTENTIAL_TYPES = ['linear', 'quadratic', 'conic', 'exponential']

# Variable global para mantener la velocidad anterior (para rampa)
_last_v_linear = 0.0


def reset_velocity_ramp():
    """Resetea la rampa de aceleración al inicio de navegación"""
    global _last_v_linear
    _last_v_linear = 0.0


def _wrap_pi(angle_rad):
    """
    Normaliza ángulo a (-π, π].
    
    Args:
        angle_rad: Ángulo en radianes
    
    Returns:
        Ángulo normalizado en (-π, π]
    """
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def attractive_wheel_speeds(q, q_goal, k_lin=None, k_ang=None, potential_type='linear'):
    """
    Calcula velocidades de ruedas usando campo de potencial atractivo.
    
    Args:
        q: Tupla (x, y, theta_deg) - posición actual del robot
        q_goal: Tupla (x_goal, y_goal) - posición objetivo
        k_lin: Ganancia lineal (usa la específica del tipo si es None)
        k_ang: Ganancia angular (usa config.K_ANGULAR si es None)
        potential_type: Tipo de potencial ['linear', 'quadratic', 'conic', 'exponential']
    
    Returns:
        (v_left, v_right, distance, info): Velocidades de ruedas (cm/s), distancia, y dict con info
    """
    # Seleccionar ganancia apropiada según el tipo de potencial
    if k_lin is None:
        if potential_type == 'linear':
            k_lin = config.K_LINEAR
        elif potential_type == 'quadratic':
            k_lin = config.K_QUADRATIC
        elif potential_type == 'conic':
            k_lin = config.K_CONIC
        elif potential_type == 'exponential':
            k_lin = config.K_EXPONENTIAL
        else:
            k_lin = config.K_LINEAR
    
    if k_ang is None:
        k_ang = config.K_ANGULAR
    
    # Calcular error de posición
    dx = q_goal[0] - q[0]
    dy = q_goal[1] - q[1]
    distance = math.hypot(dx, dy)
    
    # Ángulo actual del robot (convertir a radianes)
    theta_rad = math.radians(q[2])
    
    # Ángulo deseado hacia la meta
    desired_angle = math.atan2(dy, dx)
    
    # Error angular (normalizado)
    angle_error = _wrap_pi(desired_angle - theta_rad)
    
    # ========== SELECCIÓN DE FUNCIÓN DE POTENCIAL ==========
    if potential_type == 'linear':
        # Proporcional a distancia: F = k * d
        v_linear = k_lin * distance
        
    elif potential_type == 'quadratic':
        # Proporcional a distancia al cuadrado: F = k * d²
        v_linear = k_lin * (distance ** 2) / 10.0  # Normalizado
        
    elif potential_type == 'conic':
        # Cónico con saturación: F = k * min(d, d_max)
        d_sat = 100.0  # Distancia de saturación (cm)
        v_linear = k_lin * min(distance, d_sat) * 2.0
        
    elif potential_type == 'exponential':
        # Exponencial: F = k * (1 - e^(-d/λ))
        lambda_param = 50.0
        v_linear = k_lin * (1.0 - math.exp(-distance / lambda_param)) * 20.0
    else:
        v_linear = k_lin * distance  # Default a linear
    
    # Saturar a V_MAX
    if distance < config.TOL_DIST_CM:
        v_linear = 0.0
    else:
        v_linear = min(config.V_MAX_CM_S, v_linear)
        
        # RAMPA DE ACELERACIÓN: Limitar cambio de velocidad
        global _last_v_linear
        max_delta_v = config.ACCEL_RAMP_CM_S2 * config.CONTROL_DT
        
        if v_linear > _last_v_linear:
            # Acelerando: limitar aumento
            v_linear = min(v_linear, _last_v_linear + max_delta_v)
        
        # Aplicar velocidad mínima si está en movimiento
        if v_linear > 0 and v_linear < config.V_MIN_CM_S:
            v_linear = config.V_MIN_CM_S
        
        # Guardar para próxima iteración
        _last_v_linear = v_linear
    
    # Reducir velocidad lineal si el error angular es grande
    angle_factor = math.cos(angle_error)
    if angle_factor < 0:
        angle_factor = 0
    v_linear *= angle_factor
    
    # ========== CONTROL DE VELOCIDAD ANGULAR ==========
    omega = k_ang * angle_error
    
    # Saturar velocidad angular (rad/s)
    omega_max_rad_s = config.W_MAX_CM_S / (config.WHEEL_BASE_CM / 2.0)
    omega = max(-omega_max_rad_s, min(omega_max_rad_s, omega))
    
    # ========== CONVERSIÓN A VELOCIDADES DE RUEDA ==========
    half_base = config.WHEEL_BASE_CM / 2.0
    v_left = v_linear - half_base * omega
    v_right = v_linear + half_base * omega
    
    # Saturación final
    v_left = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_left))
    v_right = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_right))
    
    # Info adicional para logging
    info = {
        'potential_type': potential_type,
        'v_linear': v_linear,
        'omega': omega,
        'angle_error_deg': math.degrees(angle_error),
        'angle_factor': angle_factor
    }
    
    return v_left, v_right, distance, info


# ========== POTENCIAL REPULSIVO (Para Parte 3.2) ==========
def ir_sensors_to_obstacles(q, ir_sensors):
    """
    Convierte lecturas de sensores IR a posiciones estimadas de obstáculos en el plano global.
    
    CONVENCIONES DE ÁNGULOS:
    - Marco global: θ=0° apunta a +X (este), crece antihorario (estándar atan2)
    - Marco local del robot: +X=derecha, +Y=frente
    - Ángulos de sensores: medidos desde el frente (+Y local), +ángulo=derecha
    
    Args:
        q: Tupla (x, y, theta_deg) - posición actual del robot
        ir_sensors: Lista con 7 valores de sensores IR (0-4095)
    
    Returns:
        Lista de tuplas [(x_obs, y_obs, strength), ...] con obstáculos detectados
    """
    if not ir_sensors or len(ir_sensors) < 7:
        return []
    
    obstacles = []
    theta_robot_rad = math.radians(q[2])  # Orientación del robot en marco global
    
    for i in range(7):
        ir_value = ir_sensors[i]
        
        # Solo considerar lecturas significativas
        if ir_value < config.IR_THRESHOLD_DETECT:
            continue
        
        # Estimar distancia al obstáculo usando modelo de sensor IR
        # Basado en calibración real: a 5cm los valores van de ~200 a ~1400
        # dependiendo del ángulo de incidencia
        #
        # Los sensores IR siguen aproximadamente: I ∝ 1/d²
        # Calibración: I₀=1000 a d₀=5cm (valor típico frontal)
        # Entonces: I = I₀ * (d₀/d)² → d = d₀ * sqrt(I₀/I)
        
        # Valores de referencia de calibración (obstáculo a 5cm):
        # - Perpendicular máximo: ~1300-1400
        # - Frontal directo: ~900-1100
        # - Ángulo 45°: ~600-700
        # - Ángulo oblicuo: ~250-300
        
        # Usar valor de referencia conservador (frontal típico)
        I_ref = 1000.0  # Intensidad de referencia a 5cm frontal
        d_ref = 5.0     # Distancia de referencia (cm)
        
        # Modelo inverso al cuadrado con saturación
        if ir_value >= I_ref:
            # Muy cerca o perpendicular
            d_estimate = d_ref
        else:
            # Calcular distancia: d = d_ref * sqrt(I_ref / I)
            d_estimate = d_ref * math.sqrt(I_ref / max(ir_value, 50))
            
            # Limitar rango de estimación (sensor efectivo entre 5-40cm)
            d_estimate = max(5.0, min(d_estimate, 40.0))
        
        # Obtener ángulo del sensor relativo al frente del robot
        if i not in config.IR_SENSOR_ANGLES:
            continue
        
        sensor_angle_from_front_deg = config.IR_SENSOR_ANGLES[i]
        sensor_angle_from_front_rad = math.radians(sensor_angle_from_front_deg)
        
        # NOTA IMPORTANTE: En el Create 3, θ=0° significa que el robot mira hacia +X (este)
        # Por lo tanto, cuando θ=0°, el frente del robot apunta a +X
        # Los ángulos de sensores se miden desde el frente del robot
        
        # Dirección absoluta del sensor en marco global
        # Si el robot mira a θ y el sensor está a α desde el frente:
        # sensor_direction = θ + α
        sensor_direction_global = theta_robot_rad + sensor_angle_from_front_rad
        
        # Posición del sensor en marco global
        # El sensor está a distancia R del centro del robot en dirección sensor_direction
        sensor_global_x = q[0] + config.IR_SENSOR_RADIUS * math.cos(sensor_direction_global)
        sensor_global_y = q[1] + config.IR_SENSOR_RADIUS * math.sin(sensor_direction_global)
        
        # Posición del obstáculo (a distancia d_estimate desde el sensor en la misma dirección)
        obs_x = sensor_global_x + d_estimate * math.cos(sensor_direction_global)
        obs_y = sensor_global_y + d_estimate * math.sin(sensor_direction_global)
        
        obstacles.append((obs_x, obs_y, ir_value))
    
    return obstacles


def repulsive_force(q, ir_sensors, k_rep=None, d_influence=None):
    """
    Calcula fuerza repulsiva basada en sensores IR.
    
    Usa una función de potencial repulsivo del tipo:
    F_rep = k_rep * (1/d - 1/d_inf) * (1/d²) si d < d_inf
    F_rep = 0 si d >= d_inf
    
    Args:
        q: Posición actual (x, y, theta_deg)
        ir_sensors: Lista de 7 valores de sensores IR
        k_rep: Ganancia repulsiva (usa config.K_REPULSIVE si es None)
        d_influence: Distancia de influencia en cm (usa config.D_INFLUENCE si es None)
    
    Returns:
        (fx, fy): Fuerza repulsiva total en x, y
    """
    if k_rep is None:
        k_rep = config.K_REPULSIVE
    
    if d_influence is None:
        d_influence = config.D_INFLUENCE
    
    # Convertir sensores IR a obstáculos
    obstacles = ir_sensors_to_obstacles(q, ir_sensors)
    
    if not obstacles:
        return 0.0, 0.0
    
    fx_total = 0.0
    fy_total = 0.0
    
    for obs_x, obs_y, strength in obstacles:
        # Distancia del robot al obstáculo estimado
        dx = q[0] - obs_x
        dy = q[1] - obs_y
        distance = math.hypot(dx, dy)
        
        # Evitar división por cero
        if distance < 0.5:
            distance = 0.5
        
        # Solo aplicar si está dentro del rango de influencia
        if distance < d_influence:
            # Normalizar la fuerza por el valor del sensor
            # Limitar amplificación para evitar fuerzas excesivas que detengan el robot
            strength_factor = min(strength / 1000.0, 2.0)
            
            # FUERZA REPULSIVA PARA EVASIÓN (no detención)
            # Fórmula: F = k * strength * (1/d - 1/d_inf)
            # Sin factor de urgencia exponencial para permitir avance gradual
            
            magnitude = k_rep * strength_factor * (1.0/distance - 1.0/d_influence)
            
            # Dirección: alejarse del obstáculo (vector unitario)
            if distance > 0.01:
                fx_total += magnitude * (dx / distance)
                fy_total += magnitude * (dy / distance)
    
    return fx_total, fy_total


def combined_potential_speeds(q, q_goal, ir_sensors=None, k_lin=None, k_ang=None, 
                              k_rep=None, d_influence=None, potential_type='linear'):
    """
    Combina potencial atractivo y repulsivo para calcular velocidades.
    
    MÉTODO CORRECTO DE CAMPOS DE POTENCIAL:
    1. Calcular fuerza atractiva hacia el objetivo
    2. Calcular fuerza repulsiva alejándose de obstáculos
    3. Sumar fuerzas para obtener fuerza resultante
    4. Convertir fuerza resultante a velocidades de rueda
    
    CONTROL DE SEGURIDAD POR PROXIMIDAD:
    - Limita velocidad máxima dinámicamente según lecturas IR
    - Sistema de umbrales escalonados para reacción gradual
    - Garantiza tiempo de frenado suficiente ante obstáculos
    
    Args:
        q: Tupla (x, y, theta_deg) - posición actual
        q_goal: Tupla (x_goal, y_goal) - posición objetivo
        ir_sensors: Lista de 7 valores de sensores IR (None = solo atractivo)
        k_lin: Ganancia lineal atractiva (auto-seleccionada si None)
        k_ang: Ganancia angular (usa config.K_ANGULAR si None)
        k_rep: Ganancia repulsiva (usa config.K_REPULSIVE si None)
        d_influence: Distancia de influencia repulsiva (usa config.D_INFLUENCE si None)
        potential_type: Tipo de potencial atractivo
    
    Returns:
        (v_left, v_right, distance, info): Velocidades de ruedas, distancia a meta, info
    """
    # Si no hay sensores IR, usar solo potencial atractivo
    if ir_sensors is None or not ir_sensors:
        return attractive_wheel_speeds(q, q_goal, k_lin=k_lin, k_ang=k_ang, 
                                      potential_type=potential_type)
    
    # Seleccionar ganancia atractiva apropiada según el tipo de potencial
    if k_lin is None:
        if potential_type == 'linear':
            k_lin = config.K_LINEAR
        elif potential_type == 'quadratic':
            k_lin = config.K_QUADRATIC
        elif potential_type == 'conic':
            k_lin = config.K_CONIC
        elif potential_type == 'exponential':
            k_lin = config.K_EXPONENTIAL
        else:
            k_lin = config.K_LINEAR
    
    if k_ang is None:
        k_ang = config.K_ANGULAR
    
    if k_rep is None:
        k_rep = config.K_REPULSIVE
    
    if d_influence is None:
        d_influence = config.D_INFLUENCE
    
    # ========== CONTROL DE SEGURIDAD: ANÁLISIS DE SENSORES IR ==========
    # Determinar velocidad máxima permitida basada en sensores frontales
    max_ir_front = 0
    if ir_sensors and len(ir_sensors) >= 7:
        # Sensores frontales críticos: 1, 2, 3, 4 (ver calibración)
        max_ir_front = max(ir_sensors[1], ir_sensors[2], ir_sensors[3], ir_sensors[4])
    
    # Límite de velocidad dinámico por proximidad (sistema escalonado)
    if max_ir_front >= config.IR_THRESHOLD_EMERGENCY:
        # EMERGENCIA: obstáculo muy cerca (<5cm), PARAR
        v_max_allowed = config.V_MAX_EMERGENCY
        safety_level = "EMERGENCY"
    elif max_ir_front >= config.IR_THRESHOLD_CRITICAL:
        # CRÍTICO: obstáculo cerca (~5-10cm), velocidad muy reducida
        v_max_allowed = config.V_MAX_CRITICAL
        safety_level = "CRITICAL"
    elif max_ir_front >= config.IR_THRESHOLD_WARNING:
        # ADVERTENCIA: obstáculo medio (~10-20cm), velocidad reducida
        v_max_allowed = config.V_MAX_WARNING
        safety_level = "WARNING"
    elif max_ir_front >= config.IR_THRESHOLD_CAUTION:
        # PRECAUCIÓN: obstáculo lejano (~20-40cm), velocidad limitada
        v_max_allowed = config.V_MAX_CAUTION
        safety_level = "CAUTION"
    else:
        # LIBRE: sin obstáculos detectados, velocidad normal
        v_max_allowed = config.V_MAX_CM_S
        safety_level = "CLEAR"
    
    # ========== PASO 1: CALCULAR FUERZA ATRACTIVA ==========
    dx_goal = q_goal[0] - q[0]
    dy_goal = q_goal[1] - q[1]
    distance = math.hypot(dx_goal, dy_goal)
    
    # Fuerza atractiva según tipo de potencial (en dirección a la meta)
    if distance < config.TOL_DIST_CM:
        fx_att = 0.0
        fy_att = 0.0
    else:
        direction_x = dx_goal / distance
        direction_y = dy_goal / distance
        
        if potential_type == 'linear':
            f_magnitude = k_lin * distance
        elif potential_type == 'quadratic':
            f_magnitude = k_lin * (distance ** 2) / 10.0
        elif potential_type == 'conic':
            f_magnitude = k_lin * min(distance, 100.0) * 2.0
        elif potential_type == 'exponential':
            f_magnitude = k_lin * (1 - math.exp(-distance / 50.0)) * 20.0
        else:
            f_magnitude = k_lin * distance
        
        fx_att = f_magnitude * direction_x
        fy_att = f_magnitude * direction_y
    
    # ========== PASO 2: CALCULAR FUERZA REPULSIVA ==========
    fx_rep, fy_rep = repulsive_force(q, ir_sensors, k_rep=k_rep, d_influence=d_influence)
    
    # ========== ESTRATEGIA DE EVASIÓN ==========
    # NUEVO ENFOQUE: Usar velocidad del atractivo, ajustar dirección por repulsivo
    # Esto asegura que el robot SIEMPRE avance (no se quede oscilando)
    
    # Velocidad lineal base del potencial atractivo
    if distance < config.TOL_DIST_CM:
        v_base = 0.0
    else:
        if potential_type == 'linear':
            v_base = k_lin * distance
        elif potential_type == 'quadratic':
            v_base = k_lin * (distance ** 2) / 10.0
        elif potential_type == 'conic':
            v_base = k_lin * min(distance, 100.0) * 2.0
        elif potential_type == 'exponential':
            v_base = k_lin * (1 - math.exp(-distance / 50.0)) * 20.0
        else:
            v_base = k_lin * distance
        
        # LIMITACIÓN DE SEGURIDAD: aplicar v_max dinámico ANTES de otras saturaciones
        v_base = min(v_base, v_max_allowed)
        
        # Saturar a velocidad máxima configurada
        v_base = min(v_base, config.V_MAX_CM_S)
        
        # Rampa de aceleración
        global _last_v_linear
        max_accel = config.ACCEL_RAMP_CM_S2 * config.CONTROL_DT
        if v_base > _last_v_linear + max_accel:
            v_base = _last_v_linear + max_accel
        _last_v_linear = v_base
        
        # Velocidad mínima
        if v_base > 0 and v_base < config.V_MIN_CM_S:
            v_base = config.V_MIN_CM_S
    
    # Ángulo deseado hacia la meta
    desired_angle_att = math.atan2(dy_goal, dx_goal)
    
    # Si hay fuerzas repulsivas significativas, ajustar el ángulo deseado
    f_rep_mag = math.hypot(fx_rep, fy_rep)
    
    if f_rep_mag > 0.5:  # Hay obstáculos cercanos
        # Combinar dirección atractiva con dirección de evasión
        # La fuerza repulsiva modifica el ángulo deseado
        angle_rep = math.atan2(fy_rep, fx_rep)
        
        # Peso de la componente repulsiva (mayor cuando está más cerca)
        weight_rep = min(f_rep_mag / 5.0, 0.9)  # Máximo 90% de influencia
        weight_att = 1.0 - weight_rep
        
        # Combinar ángulos (promedio ponderado de vectores)
        combined_x = weight_att * math.cos(desired_angle_att) + weight_rep * math.cos(angle_rep)
        combined_y = weight_att * math.sin(desired_angle_att) + weight_rep * math.sin(angle_rep)
        desired_angle = math.atan2(combined_y, combined_x)
        
        # Reducción adicional por proximidad (más agresiva)
        # Ya tenemos v_max_allowed, pero aplicamos factor extra si hay fuerza repulsiva
        extra_slowdown = max(0.2, 1.0 - weight_rep * 0.6)  # Reduce hasta 80% adicional
        v_linear = v_base * extra_slowdown
    else:
        # Sin obstáculos, usar ángulo hacia la meta
        desired_angle = desired_angle_att
        v_linear = v_base
    
    # Error angular
    theta_rad = math.radians(q[2])
    angle_error = _wrap_pi(desired_angle - theta_rad)
    
    # Reducir velocidad si el error angular es grande
    # PERO mantener velocidad mínima para permitir maniobras de evasión
    angle_factor = math.cos(angle_error)
    if angle_factor < 0.1:  # Límite inferior
        angle_factor = 0.1
    v_linear *= angle_factor
    
    # Asegurar velocidad mínima para evasión efectiva cuando hay obstáculos
    if v_linear > 0 and v_linear < 1.0 and f_rep_mag > 1.0 and safety_level != "EMERGENCY":
        v_linear = 1.0  # Velocidad mínima BAJA (solo si no es emergencia)
    
    # Velocidad angular para corregir orientación
    omega = k_ang * angle_error
    
    # Saturar velocidad angular
    omega_max_rad_s = config.W_MAX_CM_S / (config.WHEEL_BASE_CM / 2.0)
    omega = max(-omega_max_rad_s, min(omega_max_rad_s, omega))
    
    # Convertir a velocidades de rueda
    half_base = config.WHEEL_BASE_CM / 2.0
    v_left = v_linear - half_base * omega
    v_right = v_linear + half_base * omega
    
    # Saturación final
    v_left = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_left))
    v_right = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_right))
    
    # Info para logging
    info = {
        'v_linear': v_linear,
        'omega': omega,
        'angle_error_deg': math.degrees(angle_error),
        'fx_attractive': fx_att,
        'fy_attractive': fy_att,
        'fx_repulsive': fx_rep,
        'fy_repulsive': fy_rep,
        'fx_total': fx_att + fx_rep,
        'fy_total': fy_att + fy_rep,
        'force_magnitude': math.hypot(fx_att + fx_rep, fy_att + fy_rep),
        'num_obstacles': len(ir_sensors_to_obstacles(q, ir_sensors)),
        'potential_type': potential_type,
        'safety_level': safety_level,
        'max_ir_front': max_ir_front,
        'v_max_allowed': v_max_allowed
    }
    
    return v_left, v_right, distance, info
