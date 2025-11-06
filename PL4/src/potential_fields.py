"""
Implementación de funciones de potencial atractivo y repulsivo para navegación autónoma

Autores: Alan Salazar, Yago Ramos
Fecha: 4 de noviembre de 2025
Institución: UIE Universidad Intercontinental de la Empresa
Asignatura: Robots Autónomos - Profesor Eladio Dapena
Robot SDK: irobot-edu-sdk

OBJETIVOS PRINCIPALES:

En este módulo implementamos todas las funciones relacionadas con el cálculo de
campos de potencial tanto atractivos como repulsivos. Nuestro objetivo principal
era desarrollar un sistema completo que permitiera calcular velocidades de rueda
basadas en campos de potencial, con soporte para diferentes funciones matemáticas
y capacidad de combinar fuerzas atractivas y repulsivas.

Los objetivos específicos que buscamos alcanzar incluyen:

1. Implementar cuatro variantes de campos de potencial atractivo que generen
   fuerzas proporcionales a la distancia hacia la meta, cada una con características
   distintas de aceleración y convergencia
2. Desarrollar un sistema para convertir lecturas de sensores IR en posiciones
   estimadas de obstáculos utilizando un modelo físico basado en la relación
   inversa al cuadrado
3. Calcular fuerzas repulsivas que permitan al robot evadir obstáculos sin
   detenerse completamente
4. Combinar vectorialmente las fuerzas atractivas y repulsivas para generar
   velocidades de rueda que permitan navegación fluida hacia el objetivo
5. Implementar sistemas de seguridad como rampa de aceleración y control dinámico
   de velocidad según proximidad de obstáculos
6. Proporcionar información detallada para logging que permita análisis comparativo
   entre diferentes funciones de potencial

CONFIGURACIÓN:

Este módulo utiliza los parámetros definidos en config.py para todas sus operaciones.
Las ganancias específicas de cada función de potencial están ajustadas según las
características de escala de cada función matemática. El sistema está diseñado para
trabajar con el robot Create 3 y sus siete sensores infrarrojos distribuidos alrededor
del frente del robot.

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
from . import config

# ============ CONSTANTES Y VARIABLES GLOBALES ============

# Lista de tipos de potencial disponibles para selección mediante argumentos
# Cada tipo corresponde a una función matemática diferente con características
# específicas de comportamiento
POTENTIAL_TYPES = ['linear', 'quadratic', 'conic', 'exponential']

# Variable global para mantener la velocidad lineal de la iteración anterior
# Esta variable es esencial para implementar la rampa de aceleración que previene
# cambios bruscos de velocidad que podrían causar deslizamiento o pérdida de control
_last_v_linear = 0.0


# ============ FUNCIONES AUXILIARES ============

def reset_velocity_ramp():
    """
    Resetea la rampa de aceleración al inicio de una nueva navegación.
    
    Esta función debe ser llamada al inicio de cada misión de navegación para
    asegurar que la rampa de aceleración comience desde cero. Sin este reseteo,
    la velocidad inicial podría estar limitada por el valor de una navegación
    anterior, causando comportamientos inesperados.
    """
    global _last_v_linear
    _last_v_linear = 0.0


def _wrap_pi(angle_rad):
    """
    Normaliza un ángulo al rango (-π, π] para evitar discontinuidades.
    
    Esta función es esencial para el cálculo correcto de errores angulares,
    especialmente cuando el robot necesita girar más de 180 grados. Sin esta
    normalización, los errores angulares podrían tener valores incorrectos
    que causarían giros en la dirección equivocada.
    
    Args:
        angle_rad: Ángulo en radianes a normalizar
    
    Returns:
        float: Ángulo normalizado en el rango (-π, π]
    """
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def attractive_wheel_speeds(q, q_goal, k_lin=None, k_ang=None, potential_type='linear'):
    """
    Calcula velocidades de rueda usando campo de potencial atractivo.
    
    Esta función implementa el cálculo de velocidades basado únicamente en campos
    de potencial atractivo hacia el objetivo. En cada iteración calculamos la
    distancia y dirección hacia la meta, aplicamos la función de potencial seleccionada
    para determinar la velocidad lineal deseada, y luego combinamos esta velocidad
    con una corrección angular para orientar el robot hacia el objetivo.
    
    El sistema incluye varias capas de seguridad y control:
    - Rampa de aceleración para prevenir cambios bruscos de velocidad
    - Reducción de velocidad cuando el error angular es grande
    - Saturación dentro de límites físicos del robot
    - Detección de llegada a meta
    
    Args:
        q: Tupla (x, y, theta_deg) con posición y orientación actual del robot
        q_goal: Tupla (x_goal, y_goal) con coordenadas del objetivo
        k_lin: Ganancia lineal específica (usa la del tipo de potencial si es None)
        k_ang: Ganancia angular para corrección de orientación (usa config.K_ANGULAR si es None)
        potential_type: Tipo de función de potencial ['linear', 'quadratic', 'conic', 'exponential']
    
    Returns:
        tuple: Tupla con (v_left, v_right, distance, info) donde:
            - v_left: Velocidad de rueda izquierda en cm/s
            - v_right: Velocidad de rueda derecha en cm/s
            - distance: Distancia al objetivo en cm
            - info: Diccionario con información adicional para logging
    """
    # Seleccionar la ganancia lineal apropiada según el tipo de potencial
    # Cada función tiene características diferentes de escala, por lo que requieren
    # ganancias ajustadas independientemente para lograr comportamientos similares
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
    
    # Calcular el vector de error de posición hacia el objetivo
    # Este vector nos indica la dirección y distancia que debemos recorrer
    dx = q_goal[0] - q[0]
    dy = q_goal[1] - q[1]
    distance = math.hypot(dx, dy)
    
    # Convertir la orientación actual del robot de grados a radianes
    # para realizar cálculos trigonométricos
    theta_rad = math.radians(q[2])
    
    # Calcular el ángulo deseado hacia la meta usando atan2 para obtener
    # el ángulo en el rango correcto considerando todos los cuadrantes
    desired_angle = math.atan2(dy, dx)
    
    # Calcular el error angular normalizado entre la dirección deseada y
    # la orientación actual. Esta normalización es crucial para evitar
    # giros innecesarios cuando el robot está cerca de la orientación correcta
    angle_error = _wrap_pi(desired_angle - theta_rad)
    
    # ========== CALCULAR VELOCIDAD LINEAL SEGÚN FUNCIÓN DE POTENCIAL ==========
    # Aplicamos la función de potencial seleccionada para determinar la velocidad
    # lineal deseada. Cada función tiene características diferentes:
    if potential_type == 'linear':
        # Función lineal: F = k * d
        # Proporcional directa a la distancia. Comportamiento predecible y constante
        # Mantiene velocidad aproximadamente constante una vez alcanzada la máxima
        v_linear = k_lin * distance
        
    elif potential_type == 'quadratic':
        # Función cuadrática: F = k * d² / 10
        # La fuerza crece con el cuadrado de la distancia, resultando en aceleración
        # más agresiva cuando está lejos y desaceleración suave cuando se acerca
        # El factor de normalización /10 ajusta la escala para evitar valores excesivos
        v_linear = k_lin * (distance ** 2) / 10.0
        
    elif potential_type == 'conic':
        # Función cónica con saturación: F = k * min(d, d_sat) * 2
        # Saturación a 100 cm significa que cuando el robot está más lejos,
        # la velocidad se mantiene constante. Solo cuando se acerca comienza a reducir
        # El factor *2 aumenta la velocidad base para compensar la saturación
        d_sat = 100.0  # Distancia de saturación en centímetros
        v_linear = k_lin * min(distance, d_sat) * 2.0
        
    elif potential_type == 'exponential':
        # Función exponencial: F = k * (1 - e^(-d/λ)) * 20
        # Presenta convergencia asintótica, acelerando rápidamente al inicio pero
        # desacelerando de forma muy suave conforme se acerca al objetivo
        # El factor *20 aumenta la velocidad para compensar la normalización 0-1
        lambda_param = 50.0  # Parámetro de escala para la exponencial
        v_linear = k_lin * (1.0 - math.exp(-distance / lambda_param)) * 20.0
    else:
        # Por defecto usar función lineal si el tipo no es reconocido
        v_linear = k_lin * distance
    
    # ========== APLICAR LÍMITES Y SEGURIDAD ==========
    
    # Si estamos muy cerca de la meta, detener completamente
    if distance < config.TOL_DIST_CM:
        v_linear = 0.0
    else:
        # Saturar la velocidad calculada al máximo permitido
        v_linear = min(config.V_MAX_CM_S, v_linear)
        
        # Aplicar rampa de aceleración progresiva desde arranque hasta velocidad máxima
        global _last_v_linear
        max_delta_v = config.ACCEL_RAMP_CM_S2 * config.CONTROL_DT
        
        # Solo limitar cuando estamos acelerando, no cuando desaceleramos
        if v_linear > _last_v_linear:
            # Acelerando: limitar el aumento máximo permitido
            v_linear = min(v_linear, _last_v_linear + max_delta_v)
        
        # Guardar la velocidad actual para la próxima iteración de la rampa
        _last_v_linear = v_linear
    
    # MODIFICADO: Control inteligente de velocidad angular con MOVIMIENTO EN ARCO
    # El robot SIEMPRE avanza y corrige su orientación mediante velocidades diferenciales
    # NUNCA gira sobre su propio eje
    
    # Calcular factor de reducción basado en el error angular
    angle_factor = math.cos(angle_error)
    
    # CLAVE: Definir velocidad mínima según distancia a la meta
    if distance > 50.0:
        # Lejos de la meta: mantener velocidad mínima alta para arcos suaves
        # Incluso si está apuntando en dirección contraria, debe avanzar
        min_factor = 0.6  # Mínimo 60% de velocidad calculada
    elif distance > 20.0:
        # Distancia media: permitir más reducción pero mantener avance
        min_factor = 0.4  # Mínimo 40% de velocidad
    else:
        # Cerca de la meta: permitir velocidades más bajas para convergencia precisa
        min_factor = 0.2  # Mínimo 20% de velocidad
    
    # Aplicar el factor con el mínimo garantizado
    if angle_factor < min_factor:
        angle_factor = min_factor
    
    # Aplicar reducción (pero siempre manteniendo velocidad mínima)
    v_linear *= angle_factor
    
    # GARANTÍA ADICIONAL: velocidad mínima absoluta cuando estamos lejos
    if distance > 30.0 and v_linear < 8.0:  # Mínimo 8 cm/s cuando estamos lejos
        v_linear = 8.0
    
    # ========== CALCULAR VELOCIDAD ANGULAR ==========
    # La velocidad angular es proporcional al error angular, permitiendo que
    # el robot corrija su orientación más rápidamente cuando el error es mayor
    omega = k_ang * angle_error
    
    # Convertir el límite de velocidad angular de cm/s a rad/s
    # El límite está expresado como diferencia máxima entre ruedas, por lo que
    # necesitamos dividir por la mitad de la distancia entre ruedas
    omega_max_rad_s = config.W_MAX_CM_S / (config.WHEEL_BASE_CM / 2.0)
    
    # Saturar la velocidad angular dentro del límite máximo permitido
    omega = max(-omega_max_rad_s, min(omega_max_rad_s, omega))
    
    # ========== RESTRICCIÓN PARA NAVEGACIÓN EN ARCO ==========
    # CLAVE: Limitar omega para que ambas ruedas siempre avancen (no giros sobre eje)
    # Si omega es muy grande, una rueda iría hacia atrás, causando giro en lugar de arco
    # Forzamos que la rueda más lenta siempre tenga velocidad >= MIN_WHEEL_SPEED
    half_base = config.WHEEL_BASE_CM / 2.0
    
    # Definir velocidad mínima de rueda según distancia al objetivo
    if distance > 30.0:
        # LEJOS: Mantener ambas ruedas con velocidad mínima significativa
        min_wheel_speed = 4.0  # cm/s - garantiza arco suave
    elif distance > 10.0:
        # MEDIO: Permitir velocidades de rueda más bajas
        min_wheel_speed = 2.0  # cm/s
    else:
        # CERCA: Permitir casi detenerse para convergencia precisa
        min_wheel_speed = 0.0  # cm/s
    
    # Aplicar restricción de arco siempre que no estemos en la meta
    if distance > config.TOL_DIST_CM and v_linear > min_wheel_speed:
        # Calcular el omega máximo que mantiene la rueda más lenta >= min_wheel_speed
        # Queremos: v_linear - half_base * |omega| >= min_wheel_speed
        # Por lo tanto: |omega| <= (v_linear - min_wheel_speed) / half_base
        max_omega_for_arc = (v_linear - min_wheel_speed) / half_base
        if abs(omega) > max_omega_for_arc:
            # Reducir omega para mantener navegación en arco
            omega = math.copysign(max_omega_for_arc, omega)
    
    # ========== CONVERSIÓN A VELOCIDADES INDIVIDUALES DE RUEDA ==========
    # Convertimos la velocidad lineal y angular en velocidades individuales
    # para cada rueda usando la cinemática diferencial del robot
    # v_left = v_linear - (WHEEL_BASE/2) * omega
    # v_right = v_linear + (WHEEL_BASE/2) * omega
    v_left = v_linear - half_base * omega
    v_right = v_linear + half_base * omega
    
    # Saturación final de cada rueda individualmente para garantizar que
    # nunca excedamos los límites físicos del robot, incluso si el cálculo
    # anterior produjo valores fuera de rango
    v_left = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_left))
    v_right = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_right))
    
    # Preparar información adicional para logging y análisis
    # Esta información se registra en los archivos CSV para permitir análisis
    # comparativo posterior entre diferentes funciones de potencial
    info = {
        'potential_type': potential_type,
        'v_linear': v_linear,
        'omega': omega,
        'angle_error_deg': math.degrees(angle_error),
        'angle_factor': angle_factor
    }
    
    return v_left, v_right, distance, info


# ========== FUNCIONES DE POTENCIAL REPULSIVO (Para Parte 3.2) ==========

def normalize_ir_reading(ir_value, sensor_index):
    """
    Normaliza la lectura de un sensor IR según su sensibilidad calibrada.
    
    Los sensores IR del robot tienen diferentes sensibilidades. Esta función
    normaliza las lecturas para que un obstáculo a la misma distancia produzca
    valores comparables entre sensores.
    
    Basado en calibración real:
    - Sensor 0: 1382 a 5cm (muy sensible)
    - Sensor 1: 1121 a 5cm (muy sensible)  
    - Sensor 2: 270 a 5cm (menos sensible)
    - Sensor 3: 1045 a 5cm (muy sensible)
    - Sensor 4: 896 a 5cm (sensible)
    - Sensor 5: 672 a 5cm (menos sensible)
    - Sensor 6: 901 a 5cm (sensible)
    
    Args:
        ir_value: Valor crudo del sensor IR (0-4095)
        sensor_index: Índice del sensor (0-6)
    
    Returns:
        float: Valor normalizado donde 1000 representa ~5cm para todos los sensores
    """
    if sensor_index not in config.IR_SENSOR_SENSITIVITY_FACTORS:
        return ir_value  # Sin normalización si no está calibrado
    
    factor = config.IR_SENSOR_SENSITIVITY_FACTORS[sensor_index]
    return ir_value / factor


def ir_value_to_distance(ir_value):
    """
    Convierte un valor de sensor IR (0-4095) a distancia estimada en centímetros.
    
    Utiliza el modelo físico de los sensores IR que siguen aproximadamente
    la relación: intensidad ∝ 1/distancia²
    
    Basado en calibración real:
    - A 5cm del borde: valores ~800-1400 dependiendo del sensor y ángulo
    - Los sensores frontales perpendiculares dan lecturas más altas
    - Los sensores laterales en ángulo dan lecturas más bajas
    
    Fórmula: distancia = k / sqrt(IR_value)
    donde k está calibrado experimentalmente
    
    Args:
        ir_value: Valor del sensor IR (0-4095)
    
    Returns:
        float: Distancia estimada en centímetros (entre IR_MIN_DISTANCE_CM y IR_MAX_DISTANCE_CM)
    """
    # Valores muy bajos o cero indican sin obstáculo (distancia máxima)
    if ir_value < config.IR_THRESHOLD_DETECT:
        return config.IR_MAX_DISTANCE_CM
    
    # Aplicar modelo inverso al cuadrado: d = k / sqrt(I)
    # Con k calibrado a partir de medición a 5cm con IR≈1000
    distance = config.IR_DISTANCE_CONSTANT / math.sqrt(ir_value)
    
    # Saturar al rango válido de estimación
    distance = max(config.IR_MIN_DISTANCE_CM, min(distance, config.IR_MAX_DISTANCE_CM))
    
    return distance


def detect_navigable_gaps(ir_sensors, q):
    """
    Detecta espacios navegables (gaps) entre obstáculos basándose en lecturas IR.
    
    Analiza los 7 sensores IR para identificar pares de obstáculos laterales
    con espacio suficiente entre ellos para que el robot pueda pasar. Un gap
    es navegable si:
    
    1. Hay dos sensores adyacentes o cercanos que detectan obstáculos
    2. Los sensores entre ellos reportan espacio libre
    3. El ancho del espacio es mayor que el diámetro del robot más margen
    4. El gap está en una dirección que tiene sentido para la navegación
    
    Esta función es crítica para resolver el problema de "no animarse a pasar"
    entre obstáculos, permitiendo que el robot identifique pasillos navegables.
    
    Args:
        ir_sensors: Lista de 7 valores de sensores IR (0-4095)
        q: Tupla (x, y, theta_deg) con posición y orientación del robot
    
    Returns:
        list: Lista de diccionarios describiendo gaps detectados:
            [
                {
                    'left_sensor': índice del sensor izquierdo del gap,
                    'right_sensor': índice del sensor derecho del gap,
                    'gap_angle': ángulo central del gap en el marco global (grados),
                    'gap_width': ancho estimado del gap en centímetros,
                    'is_navigable': True si el robot cabe en el gap,
                    'left_distance': distancia al obstáculo izquierdo (cm),
                    'right_distance': distancia al obstáculo derecho (cm)
                }
            ]
    """
    if not ir_sensors or len(ir_sensors) < 7:
        return []
    
    gaps = []
    theta_robot_rad = math.radians(q[2])
    
    # Convertir todas las lecturas IR a distancias
    distances = [ir_value_to_distance(ir_sensors[i]) for i in range(7)]
    
    # Analizar cada par de sensores adyacentes para buscar gaps
    for i in range(7):
        # Verificar si este sensor detecta un obstáculo
        if ir_sensors[i] < config.GAP_BLOCKED_THRESHOLD:
            continue
        
        # Buscar el siguiente sensor que también detecte un obstáculo
        # (puede no ser el inmediatamente adyacente)
        for j in range(i + 1, min(i + 4, 7)):  # Buscar hasta 3 sensores adelante
            if ir_sensors[j] < config.GAP_BLOCKED_THRESHOLD:
                continue
            
            # Tenemos dos sensores que detectan obstáculos: i y j
            # Verificar si hay espacio libre entre ellos
            
            # Verificar que los sensores intermedios estén libres
            all_clear_between = True
            for k in range(i + 1, j):
                if ir_sensors[k] >= config.GAP_CLEAR_THRESHOLD:
                    all_clear_between = False
                    break
            
            if not all_clear_between:
                continue  # No hay espacio libre entre los obstáculos
            
            # Calcular el ancho del gap basándose en geometría
            # Los obstáculos están en las direcciones de los sensores i y j
            # a las distancias estimadas
            
            angle_i = config.IR_SENSOR_ANGLES.get(i, 0)
            angle_j = config.IR_SENSOR_ANGLES.get(j, 0)
            
            dist_i = distances[i]
            dist_j = distances[j]
            
            # Posiciones de los obstáculos en el marco local del robot
            # (relativo al centro del robot)
            angle_i_rad = math.radians(angle_i)
            angle_j_rad = math.radians(angle_j)
            
            # Posiciones en marco local (x=derecha, y=frente)
            # Convertir a marco global para cálculo preciso
            obs_i_local_x = dist_i * math.sin(angle_i_rad)
            obs_i_local_y = dist_i * math.cos(angle_i_rad)
            
            obs_j_local_x = dist_j * math.sin(angle_j_rad)
            obs_j_local_y = dist_j * math.cos(angle_j_rad)
            
            # Calcular la distancia entre los dos obstáculos
            gap_width = math.hypot(
                obs_i_local_x - obs_j_local_x,
                obs_i_local_y - obs_j_local_y
            )
            
            # Verificar si el gap es navegable
            is_navigable = gap_width >= config.GAP_MIN_WIDTH_CM
            
            # Calcular el ángulo central del gap (promedio de los ángulos de los obstáculos)
            gap_angle_local = (angle_i + angle_j) / 2.0
            gap_angle_global = q[2] + gap_angle_local
            
            # Normalizar el ángulo global a [-180, 180]
            while gap_angle_global > 180:
                gap_angle_global -= 360
            while gap_angle_global < -180:
                gap_angle_global += 360
            
            # Crear el descriptor del gap
            gap_info = {
                'left_sensor': i,
                'right_sensor': j,
                'gap_angle': gap_angle_global,
                'gap_width': gap_width,
                'is_navigable': is_navigable,
                'left_distance': dist_i,
                'right_distance': dist_j,
                'sensors_between': j - i - 1  # Número de sensores libres entre obstáculos
            }
            
            gaps.append(gap_info)
            break  # Solo buscar un gap por sensor izquierdo
    
    return gaps


def ir_sensors_to_obstacles(q, ir_sensors):
    """
    Convierte lecturas de sensores IR a posiciones estimadas de obstáculos en el plano global.
    
    Esta función es fundamental para el sistema de evasión de obstáculos. Leemos
    las siete lecturas de sensores IR del robot y, utilizando un modelo físico
    basado en la relación inversa al cuadrado entre intensidad y distancia, estimamos
    dónde se encuentran los obstáculos en el plano global. Esta información se utiliza
    posteriormente para calcular fuerzas repulsivas que modifican la trayectoria del robot.
    
    El modelo físico que utilizamos está basado en calibración real realizada con
    obstáculos a diferentes distancias. Los sensores IR siguen aproximadamente la
    relación I ∝ 1/d², donde I es la intensidad medida y d es la distancia al obstáculo.
    
    CONVENCIONES DE ÁNGULOS:
    - Marco global: θ=0° apunta a +X (este), crece antihorario (estándar atan2)
    - Marco local del robot: +X=derecha, +Y=frente
    - Ángulos de sensores: medidos desde el frente (+Y local), +ángulo=derecha
    
    La transformación de coordenadas locales a globales considera tanto la posición
    del robot como su orientación, permitiendo que las fuerzas repulsivas se calculen
    correctamente independientemente de hacia dónde esté mirando el robot.
    
    Args:
        q: Tupla (x, y, theta_deg) con posición y orientación actual del robot
        ir_sensors: Lista con 7 valores de sensores IR (rango 0-4095)
    
    Returns:
        list: Lista de tuplas [(x_obs, y_obs, strength), ...] donde cada tupla contiene:
            - x_obs: Coordenada X estimada del obstáculo en el plano global (cm)
            - y_obs: Coordenada Y estimada del obstáculo en el plano global (cm)
            - strength: Valor de intensidad del sensor IR (0-4095)
    """
    # Validar que tengamos lecturas de sensores válidas
    if not ir_sensors or len(ir_sensors) < 7:
        return []
    
    obstacles = []
    # Convertir la orientación del robot a radianes para cálculos trigonométricos
    theta_robot_rad = math.radians(q[2])
    
    # Procesar cada uno de los siete sensores IR
    for i in range(7):
        ir_value = ir_sensors[i]
        
        # Solo considerar lecturas que superen el umbral mínimo de detección
        # Esto filtra ruido y lecturas de sensores que no detectan obstáculos
        if ir_value < config.IR_THRESHOLD_DETECT:
            continue
        
        # ========== ESTIMACIÓN DE DISTANCIA MEDIANTE MODELO FÍSICO ==========
        # Los sensores IR siguen aproximadamente la relación I ∝ 1/d²
        # Modelo simplificado: d = d_ref * sqrt(I_ref / I)
        # 
        # Calibración: a 5cm los valores van de ~270 a ~1400 según ángulo
        # MODIFICADO: Estimación CONSERVADORA (subestimar distancia) para mayor seguridad
        # Al pensar que los obstáculos están MÁS CERCA, genera fuerzas repulsivas MAYORES
        
        # Valores de referencia AJUSTADOS para estimación conservadora
        I_ref = 800.0   # REDUCIDO de 1000→800: estima distancias MÁS CORTAS
        d_ref = 4.0     # REDUCIDO de 5.0→4.0: asume obstáculos más cerca
        
        # Aplicar modelo inverso al cuadrado con saturación
        if ir_value >= I_ref:
            # Obstáculo muy cerca, usar distancia mínima
            d_estimate = d_ref
        else:
            # Calcular distancia: d = d_ref * sqrt(I_ref / I)
            d_estimate = d_ref * math.sqrt(I_ref / max(ir_value, 50))
            
            # Limitar al rango efectivo del sensor (4-35cm)
            # REDUCIDO: 5-40cm → 4-35cm para mantener estimaciones conservadoras
            d_estimate = max(4.0, min(d_estimate, 35.0))
        
        # ========== TRANSFORMACIÓN DE COORDENADAS ==========
        # Obtener el ángulo del sensor relativo al frente del robot desde la configuración
        if i not in config.IR_SENSOR_ANGLES:
            continue
        
        sensor_angle_from_front_deg = config.IR_SENSOR_ANGLES[i]
        sensor_angle_from_front_rad = math.radians(sensor_angle_from_front_deg)
        
        # Calcular la dirección absoluta del sensor en el marco global
        # Si el robot está orientado a θ y el sensor está a α desde el frente,
        # la dirección global del sensor es θ + α
        sensor_direction_global = theta_robot_rad + sensor_angle_from_front_rad
        
        # Calcular la posición del sensor en el marco global
        # El sensor está montado en el borde del robot (a distancia IR_SENSOR_RADIUS
        # del centro) en la dirección calculada
        sensor_global_x = q[0] + config.IR_SENSOR_RADIUS * math.cos(sensor_direction_global)
        sensor_global_y = q[1] + config.IR_SENSOR_RADIUS * math.sin(sensor_direction_global)
        
        # Calcular la posición estimada del obstáculo
        # El obstáculo está a distancia d_estimate desde el sensor en la misma dirección
        # que apunta el sensor
        obs_x = sensor_global_x + d_estimate * math.cos(sensor_direction_global)
        obs_y = sensor_global_y + d_estimate * math.sin(sensor_direction_global)
        
        # Agregar el obstáculo a la lista con su posición y fuerza de la señal
        obstacles.append((obs_x, obs_y, ir_value))
    
    return obstacles


def find_best_free_direction(ir_sensors, current_heading_deg, goal_angle_deg):
    """
    Encuentra la mejor dirección libre analizando los 7 sectores de sensores.
    
    En lugar de simplemente aplicar fuerzas repulsivas que frenan el robot,
    esta función analiza TODAS las direcciones disponibles y encuentra la
    MEJOR ruta libre que también se acerque al objetivo.
    
    Args:
        ir_sensors: Lista de 7 valores IR normalizados
        current_heading_deg: Orientación actual del robot
        goal_angle_deg: Dirección hacia el objetivo
    
    Returns:
        tuple: (best_direction_deg, clearance_score, should_slow)
            - best_direction_deg: Mejor dirección libre relativa al robot (-90 a +90)
            - clearance_score: Qué tan libre está (0.0=bloqueado, 1.0=completamente libre)
            - should_slow: True si debe reducir velocidad (peligro real)
    """
    # Normalizar sensores
    normalized_ir = [normalize_ir_reading(ir_sensors[i], i) for i in range(7)]
    
    # Definir sectores angulares para cada sensor (relativo al frente del robot)
    sensor_angles = [config.IR_SENSOR_ANGLES.get(i, 0) for i in range(7)]
    
    # Calcular "libertad" en cada dirección (invertir: alto IR = obstáculo cerca = baja libertad)
    freedom_scores = []
    for i in range(7):
        ir_norm = normalized_ir[i]
        
        # Convertir IR a "score de libertad" (0=bloqueado, 1=libre)
        if ir_norm < config.IR_THRESHOLD_DETECT:
            freedom = 1.0  # Completamente libre
        elif ir_norm >= config.IR_THRESHOLD_EMERGENCY:
            freedom = 0.0  # Bloqueado
        else:
            # Escala lineal entre DETECT y EMERGENCY
            freedom = 1.0 - (ir_norm - config.IR_THRESHOLD_DETECT) / \
                      (config.IR_THRESHOLD_EMERGENCY - config.IR_THRESHOLD_DETECT)
        
        freedom_scores.append(freedom)
    
    # Encontrar la dirección con MAYOR libertad que también se aproxime al objetivo
    error_to_goal = goal_angle_deg - current_heading_deg
    while error_to_goal > 180:
        error_to_goal -= 360
    while error_to_goal < -180:
        error_to_goal += 360
    
    best_score = -1000
    best_direction = 0
    min_freedom = 1.0
    
    for i in range(7):
        sensor_angle = sensor_angles[i]
        freedom = freedom_scores[i]
        
        # Penalizar direcciones alejadas del objetivo
        angle_diff = abs(sensor_angle - error_to_goal)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        # Score combinado: libertad (peso 70%) + cercanía al objetivo (peso 30%)
        score = 0.7 * freedom - 0.3 * (angle_diff / 180.0)
        
        if score > best_score:
            best_score = score
            best_direction = sensor_angle
            min_freedom = min(min_freedom, freedom)
    
    # Determinar si debe reducir velocidad (libertad < 50% en TODAS direcciones)
    avg_freedom = sum(freedom_scores) / len(freedom_scores)
    should_slow = avg_freedom < 0.5
    
    return best_direction, min_freedom, should_slow


def repulsive_force(q, ir_sensors, k_rep=None, d_influence=None, gaps=None):
    """
    Calcula la fuerza repulsiva DIRECCIONAL que guía hacia espacios libres.
    
    CAMBIO FUNDAMENTAL: En lugar de simplemente empujar LEJOS de obstáculos,
    esta versión IDENTIFICA la mejor dirección libre y genera una fuerza que
    GUÍA hacia esa dirección. El robot SIEMPRE encuentra por dónde pasar.
    
    Args:
        q: Tupla (x, y, theta_deg) con posición y orientación actual del robot
        ir_sensors: Lista de 7 valores de sensores IR (rango 0-4095)
        k_rep: Ganancia repulsiva (REINTERPRETADO: intensidad de corrección direccional)
        d_influence: OBSOLETO en este enfoque direccional
        gaps: OBSOLETO en este enfoque direccional
    
    Returns:
        tuple: Tupla (fx, fy) con las componentes X e Y de la fuerza DIRECCIONAL
    """
    # Usar valores por defecto de configuración si no se especificaron
    if k_rep is None:
        k_rep = config.K_REPULSIVE
    
    if d_influence is None:
        d_influence = config.D_INFLUENCE
    
    # ENFOQUE COMPLETAMENTE NUEVO: En lugar de calcular fuerzas individuales
    # de cada obstáculo (que causan frenado), identificamos la MEJOR DIRECCIÓN
    # LIBRE y generamos una fuerza que guía hacia allá
    
    # Verificar si HAY obstáculos detectados
    max_ir = max(ir_sensors) if ir_sensors else 0
    
    # Si NO hay obstáculos cercanos, no aplicar fuerza repulsiva
    if max_ir < config.IR_THRESHOLD_DETECT:
        return 0.0, 0.0
    
    # OBSTÁCULOS DETECTADOS: No frenar, sino GIRAR hacia espacio libre
    # La clave es que el robot SIEMPRE se mueve, solo cambia dirección
    
    # Analizar los 7 sensores para encontrar la dirección con MENOS obstáculos
    # Normalizar lecturas para compensar diferencias entre sensores
    normalized_ir = [normalize_ir_reading(ir_sensors[i], i) for i in range(7)]
    
    # Encontrar el sensor con MENOR lectura (más libre)
    min_ir_idx = 0
    min_ir_val = normalized_ir[0]
    
    for i in range(1, 7):
        if normalized_ir[i] < min_ir_val:
            min_ir_val = normalized_ir[i]
            min_ir_idx = i
    
    # Obtener el ángulo del sensor más libre
    free_direction_local_deg = config.IR_SENSOR_ANGLES.get(min_ir_idx, 0)
    
    # Convertir a radianes y al marco global
    theta_robot_rad = math.radians(q[2])
    free_direction_rad = theta_robot_rad + math.radians(free_direction_local_deg)
    
    # Calcular INTENSIDAD de la corrección basada en qué tan bloqueado está
    avg_ir = sum(normalized_ir) / len(normalized_ir)
    
    # Si promedio es BAJO (< DETECT), casi no hay obstáculos → fuerza débil
    # Si promedio es ALTO (> WARNING), hay muchos obstáculos → fuerza FUERTE
    if avg_ir < config.IR_THRESHOLD_DETECT:
        correction_intensity = 0.0
    elif avg_ir > config.IR_THRESHOLD_WARNING:
        correction_intensity = 1.0  # Máxima corrección
    else:
        # Escala lineal entre DETECT y WARNING
        correction_intensity = (avg_ir - config.IR_THRESHOLD_DETECT) / \
                              (config.IR_THRESHOLD_WARNING - config.IR_THRESHOLD_DETECT)
    
    # Generar fuerza que GUÍA hacia la dirección libre
    # Magnitud proporcional a: (1) ganancia k_rep, (2) intensidad de corrección
    magnitude = k_rep * correction_intensity * 0.01  # Factor 0.01 para escalar adecuadamente
    
    # Componentes de la fuerza en dirección libre
    fx_total = magnitude * math.cos(free_direction_rad)
    fy_total = magnitude * math.sin(free_direction_rad)
    
    return fx_total, fy_total



def combined_potential_speeds(q, q_goal, ir_sensors=None, k_lin=None, k_ang=None, 
                              k_rep=None, d_influence=None, potential_type='linear'):
    """
    Combina potencial atractivo y repulsivo para calcular velocidades de navegación.
    
    Esta es la función principal para la Parte 02 de la práctica, que combina campos
    de potencial atractivo y repulsivo para lograr navegación con evasión de obstáculos.
    La estrategia que implementamos utiliza la velocidad del potencial atractivo como
    base y ajusta la dirección según las fuerzas repulsivas, garantizando que el robot
    siempre avance hacia el objetivo mientras evita obstáculos.
    
    MÉTODO DE CAMPOS DE POTENCIAL COMBINADOS:
    Nuestro enfoque sigue estos pasos:
    1. Calcular fuerza atractiva hacia el objetivo usando la función de potencial seleccionada
    2. Calcular fuerza repulsiva alejándose de obstáculos detectados por sensores IR
    3. Determinar velocidad base del potencial atractivo
    4. Combinar direcciones atractiva y repulsiva mediante promedio ponderado
    5. Aplicar limitaciones de seguridad dinámicas según proximidad de obstáculos
    6. Convertir la velocidad y dirección resultantes a velocidades de rueda
    
    CONTROL DE SEGURIDAD POR PROXIMIDAD:
    El sistema implementa un control dinámico de velocidad que reduce progresivamente
    la velocidad máxima permitida según la proximidad de obstáculos detectados mediante
    sensores IR frontales. Este sistema de umbrales escalonados garantiza tiempo suficiente
    de reacción y frenado ante obstáculos, reduciendo significativamente las colisiones.
    
    Args:
        q: Tupla (x, y, theta_deg) con posición y orientación actual del robot
        q_goal: Tupla (x_goal, y_goal) con coordenadas del objetivo
        ir_sensors: Lista de 7 valores de sensores IR (None = usar solo potencial atractivo)
        k_lin: Ganancia lineal atractiva (auto-seleccionada según potential_type si es None)
        k_ang: Ganancia angular para corrección de orientación (usa config.K_ANGULAR si None)
        k_rep: Ganancia repulsiva que controla intensidad de evasión (usa config.K_REPULSIVE si None)
        d_influence: Distancia de influencia repulsiva en cm (usa config.D_INFLUENCE si None)
        potential_type: Tipo de función de potencial atractivo ['linear', 'quadratic', 'conic', 'exponential']
    
    Returns:
        tuple: Tupla con (v_left, v_right, distance, info) donde:
            - v_left: Velocidad de rueda izquierda en cm/s
            - v_right: Velocidad de rueda derecha en cm/s
            - distance: Distancia al objetivo en cm
            - info: Diccionario con información detallada para logging
    """
    # Si no hay sensores IR disponibles, usar solo potencial atractivo
    # Esto permite que la función funcione también en la Parte 01
    if ir_sensors is None or not ir_sensors:
        return attractive_wheel_speeds(q, q_goal, k_lin=k_lin, k_ang=k_ang, 
                                      potential_type=potential_type)
    
    # ========== CONFIGURACIÓN DE PARÁMETROS ==========
    # Seleccionar la ganancia atractiva apropiada según el tipo de potencial
    # Cada función requiere una ganancia específica debido a sus características de escala
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
    # IMPORTANTE: Normalizar lecturas de sensores según sensibilidad individual
    # para comparaciones justas entre sensores con diferentes características
    normalized_ir = []
    if ir_sensors and len(ir_sensors) >= 7:
        for i in range(7):
            norm_value = normalize_ir_reading(ir_sensors[i], i)
            normalized_ir.append(norm_value)
    else:
        normalized_ir = ir_sensors if ir_sensors else []
    
    # Determinar la velocidad máxima permitida basada en las lecturas NORMALIZADAS
    # CRÍTICO: Incluir sensores laterales [0] y [6] porque obstáculos laterales pueden
    # colisionar con los bordes del robot durante giros o trayectorias diagonales
    max_ir_all = 0
    max_ir_lateral = 0  # Máximo de sensores laterales extremos
    trapped_sensor_count = 0  # Contador de sensores que detectan obstáculos (trampa en C)
    
    # ========== NUEVA FUNCIONALIDAD: DETECCIÓN DE GAPS NAVEGABLES ==========
    # Detectar espacios entre obstáculos por donde el robot puede pasar
    gaps = []
    navigable_gap_detected = False
    
    if normalized_ir and len(normalized_ir) >= 7:
        # Detectar gaps usando valores NORMALIZADOS
        gaps = detect_navigable_gaps(normalized_ir, q)
        
        # Verificar si hay al menos un gap navegable
        for gap in gaps:
            if gap.get('is_navigable', False):
                navigable_gap_detected = True
                break
        
        # Considerar TODOS los sensores normalizados para detección completa
        max_ir_all = max(normalized_ir)
        # Sensores laterales extremos [0] y [6] para detección de aproximación lateral
        max_ir_lateral = max(normalized_ir[0], normalized_ir[6])
        
        # ========== DETECCIÓN DE TRAMPA EN C ==========
        # Contar cuántos sensores detectan obstáculos simultáneamente
        # Si muchos sensores están bloqueados, el robot está atrapado en una C
        # IMPORTANTE: Si hay gap navegable, NO considerar como trampa
        # Usar valores NORMALIZADOS para conteo justo
        if config.ENABLE_TRAP_ESCAPE and not navigable_gap_detected:
            for i in range(7):
                if normalized_ir[i] >= config.TRAP_DETECTION_IR_THRESHOLD:
                    trapped_sensor_count += 1
    
    # Determinar si el robot está atrapado (mínimo local)
    # MODIFICADO: No estar atrapado si hay gap navegable detectado
    is_trapped = (config.ENABLE_TRAP_ESCAPE and 
                  trapped_sensor_count >= config.TRAP_DETECTION_SENSOR_COUNT and
                  not navigable_gap_detected)
    
    # Aplicar sistema de umbrales escalonados para control dinámico de velocidad
    # MODIFICADO: Sistema MUCHO MÁS PERMISIVO que solo reduce velocidad
    # cuando hay peligro REAL (muy cerca), NO a 30cm que es completamente seguro
    
    # NUEVA LÓGICA: Solo considerar obstáculos FRONTALES directos
    # Los sensores laterales NO deben frenar el robot a menos que estén DELANTE
    front_sensors = [normalized_ir[2], normalized_ir[3], normalized_ir[4]]  # Centro y frente-laterales
    max_ir_front = max(front_sensors)
    
    # Sistema de umbrales MUY reducido: solo frenar cuando hay peligro REAL
    if max_ir_front >= config.IR_THRESHOLD_EMERGENCY:
        # EMERGENCIA: obstáculo MUY cerca (<10cm frontal), reducir a 50%
        v_max_allowed = config.V_MAX_CM_S * 0.5
        safety_level = "EMERGENCY"
    elif max_ir_front >= config.IR_THRESHOLD_CRITICAL:
        # CRÍTICO: obstáculo cerca (~10-15cm frontal), reducir a 70%
        v_max_allowed = config.V_MAX_CM_S * 0.7
        safety_level = "CRITICAL"
    elif max_ir_front >= config.IR_THRESHOLD_WARNING:
        # ADVERTENCIA: obstáculo medio (~15-25cm frontal), reducir a 85%
        v_max_allowed = config.V_MAX_CM_S * 0.85
        safety_level = "WARNING"
    else:
        # LIBRE: sin obstáculos frontales cercanos, velocidad COMPLETA
        v_max_allowed = config.V_MAX_CM_S
        safety_level = "CLEAR"
    
    # Si está atrapado, actualizar el nivel de seguridad
    if is_trapped:
        safety_level = "TRAPPED"
    
    # ========== PASO 1: CALCULAR FUERZA ATRACTIVA ==========
    # Calcular el vector de error hacia el objetivo
    dx_goal = q_goal[0] - q[0]
    dy_goal = q_goal[1] - q[1]
    distance = math.hypot(dx_goal, dy_goal)
    
    # ========== MODO ESCAPE DE TRAMPA EN C ==========
    # Si estamos atrapados, reducir la influencia del objetivo para permitir
    # que la fuerza repulsiva domine y encuentre un camino de escape
    k_lin_effective = k_lin
    k_rep_effective = k_rep
    
    # NUEVO: Boost adicional de repulsión si hay obstáculos FRONTALES críticos
    # Esto hace que el robot reaccione MÁS AGRESIVAMENTE ante obstáculos directos
    # USAR VALORES NORMALIZADOS para comparación justa
    if normalized_ir and len(normalized_ir) >= 7:
        # Sensores frontales críticos: 2, 3, 4 (los que apuntan hacia adelante)
        max_frontal = max(normalized_ir[2], normalized_ir[3], normalized_ir[4])
        if max_frontal >= config.IR_THRESHOLD_CRITICAL:
            # Si hay obstáculo frontal crítico, DUPLICAR la fuerza repulsiva
            # REDUCIDO de 3.0x a 2.0x para evitar dominación
            k_rep_effective = k_rep * 2.0
        elif max_frontal >= config.IR_THRESHOLD_WARNING:
            # Si hay obstáculo frontal en advertencia, aumentar 50%
            # REDUCIDO de 2.0x a 1.5x
            k_rep_effective = k_rep * 1.5
    
    if is_trapped:
        # Reducir atracción hacia el objetivo (permite explorar alternativas)
        k_lin_effective = k_lin * config.TRAP_ATTRACTIVE_REDUCTION
        # Aumentar repulsión adicional (empuja más fuerte lejos de obstáculos)
        # Nota: Se multiplica sobre el k_rep_effective que ya puede estar boosted
        k_rep_effective = k_rep_effective * config.TRAP_REPULSIVE_BOOST
    
    # Calcular las componentes de la fuerza atractiva según el tipo de potencial
    # Si estamos muy cerca de la meta, no hay fuerza atractiva
    if distance < config.TOL_DIST_CM:
        fx_att = 0.0
        fy_att = 0.0
    else:
        # Calcular vector unitario en dirección hacia la meta
        direction_x = dx_goal / distance
        direction_y = dy_goal / distance
        
        # Calcular la magnitud de la fuerza según la función de potencial seleccionada
        # Las fórmulas son las mismas que en attractive_wheel_speeds pero calculamos
        # la fuerza en lugar de la velocidad directamente
        if potential_type == 'linear':
            f_magnitude = k_lin_effective * distance
        elif potential_type == 'quadratic':
            f_magnitude = k_lin_effective * (distance ** 2) / 10.0
        elif potential_type == 'conic':
            f_magnitude = k_lin_effective * min(distance, 100.0) * 2.0
        elif potential_type == 'exponential':
            f_magnitude = k_lin_effective * (1 - math.exp(-distance / 50.0)) * 20.0
        else:
            f_magnitude = k_lin_effective * distance
        
        # Convertir la magnitud en componentes vectoriales
        fx_att = f_magnitude * direction_x
        fy_att = f_magnitude * direction_y
    
    # ========== PASO 2: CALCULAR FUERZA REPULSIVA ==========
    # Calcular las componentes de la fuerza repulsiva total sumando las contribuciones
    # de todos los obstáculos detectados por los sensores IR
    # MODIFICADO: Pasar información de gaps para reducir fuerzas en gaps navegables
    fx_rep, fy_rep = repulsive_force(q, ir_sensors, k_rep=k_rep_effective, 
                                     d_influence=d_influence, gaps=gaps)
    
    # ========== ESTRATEGIA DE EVASIÓN COMBINADA ==========
    # Nuestro enfoque utiliza la velocidad del potencial atractivo como base y ajusta
    # la dirección según las fuerzas repulsivas. Esto asegura que el robot SIEMPRE
    # avance hacia el objetivo (no se quede oscilando) mientras evita obstáculos
    # modificando su trayectoria de forma suave
    
    # ========== CALCULAR VELOCIDAD BASE DEL POTENCIAL ATRACTIVO ==========
    # Calculamos la velocidad lineal base usando la función de potencial atractivo
    # Esta velocidad determina qué tan rápido queremos avanzar hacia el objetivo
    if distance < config.TOL_DIST_CM:
        v_base = 0.0
    else:
        # Aplicar la función de potencial seleccionada para obtener velocidad base
        if potential_type == 'linear':
            v_base = k_lin_effective * distance
        elif potential_type == 'quadratic':
            v_base = k_lin_effective * (distance ** 2) / 10.0
        elif potential_type == 'conic':
            v_base = k_lin_effective * min(distance, 100.0) * 2.0
        elif potential_type == 'exponential':
            v_base = k_lin_effective * (1 - math.exp(-distance / 50.0)) * 20.0
        else:
            v_base = k_lin_effective * distance
        
        # LIMITACIÓN DE SEGURIDAD CRÍTICA: aplicar v_max dinámico ANTES de otras saturaciones
        # Este límite dinámico es esencial para garantizar tiempo suficiente de frenado
        # ante obstáculos detectados. Se aplica primero para que otras saturaciones no
        # lo sobrescriban
        v_base = min(v_base, v_max_allowed)
        
        # Saturar también a la velocidad máxima configurada del sistema
        v_base = min(v_base, config.V_MAX_CM_S)
        
        # ========== MODO ESCAPE: GARANTIZAR MOVIMIENTO MÍNIMO ==========
        # Si estamos atrapados, mantener velocidad mínima para seguir explorando
        if is_trapped and v_base < config.TRAP_MIN_FORWARD_SPEED:
            v_base = config.TRAP_MIN_FORWARD_SPEED
        
        # Aplicar rampa de aceleración para prevenir cambios bruscos de velocidad
        global _last_v_linear
        max_accel = config.ACCEL_RAMP_CM_S2 * config.CONTROL_DT
        if v_base > _last_v_linear + max_accel:
            v_base = _last_v_linear + max_accel
        _last_v_linear = v_base
    
    # ========== COMBINAR DIRECCIONES ATRACTIVA Y REPULSIVA ==========
    # Calcular el ángulo deseado hacia la meta desde el potencial atractivo
    desired_angle_att = math.atan2(dy_goal, dx_goal)
    
    # Calcular la magnitud de la fuerza repulsiva para determinar su influencia
    f_rep_mag = math.hypot(fx_rep, fy_rep)
    
    # Si hay fuerzas repulsivas significativas, combinamos las direcciones
    if f_rep_mag > 0.5:  # Umbral para considerar que hay obstáculos cercanos
        # Calcular el ángulo de la dirección de evasión desde la fuerza repulsiva
        angle_rep = math.atan2(fy_rep, fx_rep)
        
        # Calcular pesos para combinar las direcciones
        # El peso repulsivo aumenta cuando el robot está más cerca de obstáculos
        # CRÍTICO: Divisor DRASTICAMENTE reducido de 2.5 → 1.2 y límite aumentado de 0.98 → 0.99
        # Esto permite que la fuerza repulsiva DOMINE COMPLETAMENTE cuando hay obstáculos
        # Con divisor 1.2, fuerzas repulsivas moderadas ya alcanzan 80-90% de influencia
        weight_rep = min(f_rep_mag / 1.2, 0.99)
        weight_att = 1.0 - weight_rep
        
        # Combinar ángulos mediante promedio ponderado de vectores unitarios
        # Esto produce una dirección resultante que evita obstáculos mientras
        # mantiene el objetivo de avanzar hacia la meta
        combined_x = weight_att * math.cos(desired_angle_att) + weight_rep * math.cos(angle_rep)
        combined_y = weight_att * math.sin(desired_angle_att) + weight_rep * math.sin(angle_rep)
        desired_angle = math.atan2(combined_y, combined_x)
        
        # ⚠️ CRÍTICO: FRENAR DRÁSTICAMENTE cuando hay obstáculos
        # La velocidad debe ser INVERSAMENTE proporcional a la fuerza repulsiva
        # Cuanto más fuerte la repulsión, MÁS LENTO debe ir para girar con precisión
        # ANTES: extra_slowdown = max(0.6, 1.0 - weight_rep * 0.2)  → Muy permisivo
        # AHORA: Reducción AGRESIVA basada directamente en weight_rep
        if weight_rep > 0.8:  # Obstáculo MUY cerca
            # Con 80%+ de influencia repulsiva → ARRASTRARSE
            # weight_rep=0.9 → factor=0.1 (10% velocidad)
            # weight_rep=0.95 → factor=0.05 (5% velocidad)
            extra_slowdown = max(0.05, 1.0 - weight_rep)
        elif weight_rep > 0.5:  # Obstáculo cerca
            # Con 50-80% influencia → Reducción significativa
            # weight_rep=0.6 → factor=0.4 (40% velocidad)
            extra_slowdown = max(0.2, 1.0 - weight_rep * 0.8)
        else:  # Obstáculo detectado pero lejos
            # Con <50% influencia → Reducción moderada
            extra_slowdown = max(0.5, 1.0 - weight_rep * 0.5)
        
        v_linear = v_base * extra_slowdown
    else:
        # Sin obstáculos detectados, usar directamente el ángulo hacia la meta
        desired_angle = desired_angle_att
        v_linear = v_base
    
    # ========== CALCULAR ERROR ANGULAR Y AJUSTAR VELOCIDAD ==========
    # Calcular el error entre la dirección deseada y la orientación actual del robot
    theta_rad = math.radians(q[2])
    angle_error = _wrap_pi(desired_angle - theta_rad)
    
    # ⚠️ CRÍTICO: SISTEMA DE PROHIBICIÓN DE GIRO HACIA OBSTÁCULOS LATERALES
    # Si hay obstáculos laterales detectados con intensidad alta, PROHIBIR giros
    # hacia ese lado para evitar que el diámetro del robot colisione durante el giro
    if ir_sensors and len(ir_sensors) >= 7:
        # Sensores laterales izquierdos: 0 (esquina), 1 (intermedio)
        max_left_lateral = max(ir_sensors[0], ir_sensors[1])
        # Sensores laterales derechos: 5 (intermedio), 6 (esquina)
        max_right_lateral = max(ir_sensors[5], ir_sensors[6])
        
        # Si hay obstáculo lateral CRÍTICO y queremos girar hacia ese lado, CORREGIR dirección
        if max_left_lateral >= config.IR_THRESHOLD_CRITICAL and angle_error > 0.3:
            # Obstáculo a la IZQUIERDA y queremos girar a la IZQUIERDA (angle_error > 0)
            # FORZAR giro a la derecha o mantener curso recto
            angle_error = min(angle_error, 0.1)  # Limitar giro izquierdo a casi 0
            
        elif max_right_lateral >= config.IR_THRESHOLD_CRITICAL and angle_error < -0.3:
            # Obstáculo a la DERECHA y queremos girar a la DERECHA (angle_error < 0)
            # FORZAR giro a la izquierda o mantener curso recto
            angle_error = max(angle_error, -0.1)  # Limitar giro derecho a casi 0
        
        # Reducción adicional DRÁSTICA si hay obstáculos laterales en ADVERTENCIA
        elif max_left_lateral >= config.IR_THRESHOLD_WARNING and angle_error > 0.5:
            # Reducir el giro izquierdo a la mitad
            angle_error *= 0.5
            
        elif max_right_lateral >= config.IR_THRESHOLD_WARNING and angle_error < -0.5:
            # Reducir el giro derecho a la mitad
            angle_error *= 0.5
    
    # MODIFICADO: Control inteligente con MOVIMIENTO EN ARCO GARANTIZADO
    # El robot SIEMPRE avanza y corrige mediante velocidades diferenciales
    # NUNCA gira sobre su propio eje
    
    # Calcular factor de reducción basado en el error angular
    angle_factor = math.cos(angle_error)
    
    # CLAVE: Definir velocidad mínima según distancia a la meta
    if distance > 50.0:
        # Lejos de la meta: mantener velocidad mínima alta para arcos suaves
        # Incluso si está apuntando en dirección contraria, debe avanzar
        min_factor = 0.6  # Mínimo 60% de velocidad calculada
    elif distance > 20.0:
        # Distancia media: permitir más reducción pero mantener avance
        min_factor = 0.4  # Mínimo 40% de velocidad
    else:
        # Cerca de la meta: permitir velocidades más bajas para convergencia precisa
        min_factor = 0.2  # Mínimo 20% de velocidad
    
    # Aplicar el factor con el mínimo garantizado
    if angle_factor < min_factor:
        angle_factor = min_factor
    
    # Aplicar reducción (pero siempre manteniendo velocidad mínima)
    v_linear *= angle_factor
    
    # ⚠️ CRÍTICO: Reducción adicional si hay obstáculos LATERALES muy cerca
    # Los obstáculos laterales son especialmente peligrosos durante giros
    if ir_sensors and len(ir_sensors) >= 7:
        max_lateral_all = max(ir_sensors[0], ir_sensors[1], ir_sensors[5], ir_sensors[6])
        
        if max_lateral_all >= config.IR_THRESHOLD_CRITICAL:
            # Obstáculo lateral CRÍTICO → Reducir a 20% de velocidad
            v_linear *= 0.2
        elif max_lateral_all >= config.IR_THRESHOLD_WARNING:
            # Obstáculo lateral ADVERTENCIA → Reducir a 40% de velocidad
            v_linear *= 0.4
        elif max_lateral_all >= config.IR_THRESHOLD_CAUTION:
            # Obstáculo lateral PRECAUCIÓN → Reducir a 70% de velocidad
            v_linear *= 0.7
    
    # GARANTÍA ADICIONAL: velocidad mínima absoluta cuando estamos lejos Y sin obstáculos críticos
    if distance > 30.0 and v_linear < 8.0 and safety_level == "CLEAR":
        v_linear = 8.0
    
    # ========== CALCULAR VELOCIDAD ANGULAR ==========
    # La velocidad angular es proporcional al error angular, permitiendo corrección
    # de orientación más rápida cuando el error es mayor. Esto es especialmente
    # importante durante evasión de obstáculos cuando necesitamos cambiar dirección rápidamente
    
    # CRÍTICO: Aumentar ganancia angular cuando hay obstáculos laterales cercanos
    # Los obstáculos detectados por sensores laterales [0] y [6] requieren giros
    # más agresivos para evitar colisión con los bordes del robot
    k_ang_adjusted = k_ang
    
    # ========== MODO ESCAPE: AUMENTAR CAPACIDAD DE GIRO ==========
    # Si estamos atrapados, necesitamos poder girar más rápido para encontrar salida
    if is_trapped:
        k_ang_adjusted = k_ang * config.TRAP_ANGULAR_BOOST
    
    # Boost por obstáculos laterales cercanos (solo si no estamos en modo trampa)
    elif max_ir_lateral >= config.IR_THRESHOLD_CRITICAL:
        # Obstáculo lateral CRÍTICO: aumentar ganancia angular 50%
        k_ang_adjusted = k_ang * 1.5
    elif max_ir_lateral >= config.IR_THRESHOLD_WARNING:
        # Obstáculo lateral en ADVERTENCIA: aumentar ganancia angular 25%
        k_ang_adjusted = k_ang * 1.25
    
    # MEJORA: Reducir ganancia angular cerca del objetivo para evitar oscilación
    # Cuando estamos muy cerca (< 15cm), reducimos k_ang progresivamente para
    # permitir convergencia suave sin oscilaciones (pero solo si NO hay obstáculos laterales)
    if distance < 15.0 and max_ir_lateral < config.IR_THRESHOLD_CAUTION and not is_trapped:
        # Reducir k_ang linealmente de 100% a 30% cuando dist va de 15cm a 5cm
        reduction_factor = 0.3 + 0.7 * ((distance - 5.0) / 10.0)
        reduction_factor = max(0.3, min(1.0, reduction_factor))
        k_ang_adjusted = k_ang_adjusted * reduction_factor
    
    omega = k_ang_adjusted * angle_error
    
    # Convertir el límite de velocidad angular y saturar
    omega_max_rad_s = config.W_MAX_CM_S / (config.WHEEL_BASE_CM / 2.0)
    omega = max(-omega_max_rad_s, min(omega_max_rad_s, omega))
    
    # ========== RESTRICCIÓN PARA NAVEGACIÓN EN ARCO ==========
    # CLAVE: Limitar omega para que ambas ruedas siempre avancen (no giros sobre eje)
    # Si omega es muy grande, una rueda iría hacia atrás, causando giro en lugar de arco
    # Forzamos que la rueda más lenta siempre tenga velocidad >= MIN_WHEEL_SPEED
    half_base = config.WHEEL_BASE_CM / 2.0
    
    # Definir velocidad mínima de rueda según distancia al objetivo
    if distance > 30.0:
        # LEJOS: Mantener ambas ruedas con velocidad mínima significativa
        min_wheel_speed = 4.0  # cm/s - garantiza arco suave
    elif distance > 10.0:
        # MEDIO: Permitir velocidades de rueda más bajas
        min_wheel_speed = 2.0  # cm/s
    else:
        # CERCA: Permitir casi detenerse para convergencia precisa
        min_wheel_speed = 0.0  # cm/s
    
    # Aplicar restricción de arco siempre que no estemos en la meta
    if distance > config.TOL_DIST_CM and v_linear > min_wheel_speed:
        # Calcular el omega máximo que mantiene la rueda más lenta >= min_wheel_speed
        # Queremos: v_linear - half_base * |omega| >= min_wheel_speed
        # Por lo tanto: |omega| <= (v_linear - min_wheel_speed) / half_base
        max_omega_for_arc = (v_linear - min_wheel_speed) / half_base
        if abs(omega) > max_omega_for_arc:
            # Reducir omega para mantener navegación en arco
            omega = math.copysign(max_omega_for_arc, omega)
    
    # ========== CONVERSIÓN A VELOCIDADES DE RUEDA ==========
    # Convertir velocidad lineal y angular en velocidades individuales de rueda
    # usando la cinemática diferencial del robot
    v_left = v_linear - half_base * omega
    v_right = v_linear + half_base * omega
    
    # Saturación final de cada rueda para garantizar límites físicos
    v_left = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_left))
    v_right = max(-config.V_MAX_CM_S, min(config.V_MAX_CM_S, v_right))
    
    # ========== PREPARAR INFORMACIÓN PARA LOGGING ==========
    # Recopilar información detallada sobre el estado del sistema para análisis posterior
    # Esta información se registra en archivos CSV y permite análisis comparativo
    # entre diferentes funciones de potencial y configuraciones
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
        'max_ir_all': max_ir_all,
        'is_trapped': is_trapped,
        'trapped_sensor_count': trapped_sensor_count,
        'max_ir_lateral': max_ir_lateral,
        'v_max_allowed': v_max_allowed,
        # Información sobre gaps navegables detectados
        'num_gaps': len(gaps),
        'navigable_gap_detected': navigable_gap_detected,
        'gap_widths': [gap.get('gap_width', 0) for gap in gaps] if gaps else [],
        'gap_angles': [gap.get('gap_angle', 0) for gap in gaps] if gaps else []
    }

    
    return v_left, v_right, distance, info