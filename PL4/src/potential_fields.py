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
        
        # RAMPA DE ACELERACIÓN: Limitar el cambio de velocidad por iteración
        # Esto previene cambios bruscos que podrían causar deslizamiento o pérdida
        # de control. El límite está basado en la aceleración máxima configurada
        global _last_v_linear
        max_delta_v = config.ACCEL_RAMP_CM_S2 * config.CONTROL_DT
        
        # Solo limitar cuando estamos acelerando, no cuando desaceleramos
        if v_linear > _last_v_linear:
            # Acelerando: limitar el aumento máximo permitido
            v_linear = min(v_linear, _last_v_linear + max_delta_v)
        
        # Aplicar velocidad mínima si el robot está en movimiento pero la velocidad
        # calculada es muy baja. Esto evita que el robot se quede oscilando cerca
        # de velocidades muy pequeñas
        if v_linear > 0 and v_linear < config.V_MIN_CM_S:
            v_linear = config.V_MIN_CM_S
        
        # Guardar la velocidad actual para la próxima iteración de la rampa
        _last_v_linear = v_linear
    
    # Reducir velocidad lineal cuando el error angular es grande
    # Usamos el coseno del error angular como factor: cuando el robot está
    # orientado hacia la meta (error pequeño), el factor es cercano a 1.
    # Cuando está perpendicular o en dirección opuesta, el factor se reduce
    angle_factor = math.cos(angle_error)
    if angle_factor < 0:
        # Si el factor es negativo, significa que estamos apuntando en dirección
        # opuesta. En este caso lo establecemos a 0 para evitar movimiento hacia atrás
        angle_factor = 0
    v_linear *= angle_factor
    
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
    
    # ========== CONVERSIÓN A VELOCIDADES INDIVIDUALES DE RUEDA ==========
    # Convertimos la velocidad lineal y angular en velocidades individuales
    # para cada rueda usando la cinemática diferencial del robot
    # v_left = v_linear - (WHEEL_BASE/2) * omega
    # v_right = v_linear + (WHEEL_BASE/2) * omega
    half_base = config.WHEEL_BASE_CM / 2.0
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
        # Los sensores IR siguen aproximadamente la relación I ∝ 1/d², donde
        # I es la intensidad medida y d es la distancia al obstáculo. Esta relación
        # está basada en calibración real realizada con obstáculos a diferentes distancias.
        #
        # Calibración realizada: a 5cm los valores van de ~200 a ~1400 dependiendo
        # del ángulo de incidencia:
        # - Perpendicular máximo: ~1300-1400
        # - Frontal directo: ~900-1100
        # - Ángulo 45°: ~600-700
        # - Ángulo oblicuo: ~250-300
        #
        # Modelo: I = I₀ * (d₀/d)² → d = d₀ * sqrt(I₀/I)
        # donde I₀ es la intensidad de referencia a distancia d₀
        
        # Valores de referencia conservadores basados en calibración frontal típica
        I_ref = 1000.0  # Intensidad de referencia a 5cm frontal
        d_ref = 5.0     # Distancia de referencia en centímetros
        
        # Aplicar modelo inverso al cuadrado con saturación para evitar estimaciones
        # erróneas cuando los valores son muy altos (obstáculo muy cerca)
        if ir_value >= I_ref:
            # Si la lectura es mayor o igual a la referencia, asumimos que el
            # obstáculo está muy cerca o el sensor está perpendicular, por lo que
            # usamos la distancia mínima de referencia
            d_estimate = d_ref
        else:
            # Calcular distancia usando el modelo: d = d_ref * sqrt(I_ref / I)
            # El max(ir_value, 50) previene división por valores muy pequeños
            d_estimate = d_ref * math.sqrt(I_ref / max(ir_value, 50))
            
            # Limitar el rango de estimación al rango efectivo del sensor (5-40cm)
            # Valores fuera de este rango son poco confiables
            d_estimate = max(5.0, min(d_estimate, 40.0))
        
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


def repulsive_force(q, ir_sensors, k_rep=None, d_influence=None):
    """
    Calcula la fuerza repulsiva total basada en obstáculos detectados por sensores IR.
    
    Esta función implementa el campo de potencial repulsivo que genera fuerzas que
    alejan al robot de los obstáculos detectados. Convertimos primero las lecturas
    de sensores IR en posiciones estimadas de obstáculos, y luego calculamos fuerzas
    repulsivas para cada obstáculo que esté dentro del rango de influencia.
    
    La función de potencial repulsivo que utilizamos está diseñada para EVASIÓN
    efectiva sin detener completamente el robot. La fórmula F = k * strength * (1/d - 1/d_inf)
    genera fuerzas que crecen cuando el robot se acerca a un obstáculo, pero solo
    dentro del rango de influencia. Fuera de este rango, no hay fuerza repulsiva.
    
    La fuerza total es la suma vectorial de todas las fuerzas individuales de cada
    obstáculo detectado, lo que permite que el robot evite múltiples obstáculos
    simultáneamente.
    
    Args:
        q: Tupla (x, y, theta_deg) con posición y orientación actual del robot
        ir_sensors: Lista de 7 valores de sensores IR (rango 0-4095)
        k_rep: Ganancia repulsiva que controla la intensidad de la evasión
               (usa config.K_REPULSIVE si es None)
        d_influence: Distancia de influencia en cm. Obstáculos más lejos que esta
                     distancia no generan fuerzas repulsivas (usa config.D_INFLUENCE si es None)
    
    Returns:
        tuple: Tupla (fx, fy) con las componentes X e Y de la fuerza repulsiva total
    """
    # Usar valores por defecto de configuración si no se especificaron
    if k_rep is None:
        k_rep = config.K_REPULSIVE
    
    if d_influence is None:
        d_influence = config.D_INFLUENCE
    
    # Convertir las lecturas de sensores IR en posiciones estimadas de obstáculos
    # Esta función utiliza el modelo físico para estimar dónde están los obstáculos
    obstacles = ir_sensors_to_obstacles(q, ir_sensors)
    
    # Si no hay obstáculos detectados, no hay fuerza repulsiva
    if not obstacles:
        return 0.0, 0.0
    
    # Acumuladores para las componentes X e Y de la fuerza repulsiva total
    fx_total = 0.0
    fy_total = 0.0
    
    # Calcular fuerza repulsiva para cada obstáculo detectado
    for obs_x, obs_y, strength in obstacles:
        # Calcular el vector desde el obstáculo hacia el robot
        # La fuerza repulsiva debe alejar al robot del obstáculo, por lo que
        # apuntamos en dirección opuesta al obstáculo
        dx = q[0] - obs_x
        dy = q[1] - obs_y
        distance = math.hypot(dx, dy)
        
        # Evitar división por cero en cálculos posteriores estableciendo una
        # distancia mínima si el cálculo resulta en valor muy pequeño
        if distance < 0.5:
            distance = 0.5
        
        # Solo aplicar fuerza repulsiva si el obstáculo está dentro del rango
        # de influencia. Obstáculos más lejos no afectan el movimiento del robot
        if distance < d_influence:
            # Normalizar la fuerza por la intensidad del sensor
            # Valores más altos de intensidad indican obstáculos más cercanos o
            # con mejor reflectividad, por lo que deben generar fuerzas mayores
            # AJUSTADO: Límite aumentado de 2.0 → 3.5 para respuesta más enérgica
            # Con calibración real: valores de 1000+ a 5cm requieren amplificación mayor
            strength_factor = min(strength / 800.0, 3.5)
            
            # Calcular la magnitud de la fuerza repulsiva
            # Fórmula: F = k_rep * strength_factor * (1/d - 1/d_influence)
            # Esta fórmula genera fuerzas que crecen cuando d disminuye, pero solo
            # dentro del rango de influencia. El término (1/d - 1/d_influence) asegura
            # que la fuerza sea cero en el límite del rango de influencia
            magnitude = k_rep * strength_factor * (1.0/distance - 1.0/d_influence)
            
            # Convertir la magnitud en componentes vectoriales
            # La dirección es alejarse del obstáculo (vector unitario desde obstáculo hacia robot)
            if distance > 0.01:
                fx_total += magnitude * (dx / distance)
                fy_total += magnitude * (dy / distance)
    
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
    # Determinar la velocidad máxima permitida basada en las lecturas de sensores IR frontales
    # Los sensores frontales (1, 2, 3, 4) son los más críticos porque detectan obstáculos
    # directamente en la trayectoria del robot. Utilizamos el valor máximo de estos sensores
    # para determinar el nivel de seguridad actual
    max_ir_front = 0
    if ir_sensors and len(ir_sensors) >= 7:
        # Sensores frontales críticos según calibración (ver config.py)
        max_ir_front = max(ir_sensors[1], ir_sensors[2], ir_sensors[3], ir_sensors[4])
    
    # Aplicar sistema de umbrales escalonados para control dinámico de velocidad
    # Este sistema reduce progresivamente la velocidad máxima permitida según la
    # proximidad de obstáculos, garantizando tiempo suficiente de reacción
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
    # Calcular el vector de error hacia el objetivo
    dx_goal = q_goal[0] - q[0]
    dy_goal = q_goal[1] - q[1]
    distance = math.hypot(dx_goal, dy_goal)
    
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
            f_magnitude = k_lin * distance
        elif potential_type == 'quadratic':
            f_magnitude = k_lin * (distance ** 2) / 10.0
        elif potential_type == 'conic':
            f_magnitude = k_lin * min(distance, 100.0) * 2.0
        elif potential_type == 'exponential':
            f_magnitude = k_lin * (1 - math.exp(-distance / 50.0)) * 20.0
        else:
            f_magnitude = k_lin * distance
        
        # Convertir la magnitud en componentes vectoriales
        fx_att = f_magnitude * direction_x
        fy_att = f_magnitude * direction_y
    
    # ========== PASO 2: CALCULAR FUERZA REPULSIVA ==========
    # Calcular las componentes de la fuerza repulsiva total sumando las contribuciones
    # de todos los obstáculos detectados por los sensores IR
    fx_rep, fy_rep = repulsive_force(q, ir_sensors, k_rep=k_rep, d_influence=d_influence)
    
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
            v_base = k_lin * distance
        elif potential_type == 'quadratic':
            v_base = k_lin * (distance ** 2) / 10.0
        elif potential_type == 'conic':
            v_base = k_lin * min(distance, 100.0) * 2.0
        elif potential_type == 'exponential':
            v_base = k_lin * (1 - math.exp(-distance / 50.0)) * 20.0
        else:
            v_base = k_lin * distance
        
        # LIMITACIÓN DE SEGURIDAD CRÍTICA: aplicar v_max dinámico ANTES de otras saturaciones
        # Este límite dinámico es esencial para garantizar tiempo suficiente de frenado
        # ante obstáculos detectados. Se aplica primero para que otras saturaciones no
        # lo sobrescriban
        v_base = min(v_base, v_max_allowed)
        
        # Saturar también a la velocidad máxima configurada del sistema
        v_base = min(v_base, config.V_MAX_CM_S)
        
        # Aplicar rampa de aceleración para prevenir cambios bruscos de velocidad
        global _last_v_linear
        max_accel = config.ACCEL_RAMP_CM_S2 * config.CONTROL_DT
        if v_base > _last_v_linear + max_accel:
            v_base = _last_v_linear + max_accel
        _last_v_linear = v_base
        
        # Aplicar velocidad mínima si el robot está en movimiento pero la velocidad
        # calculada es muy baja
        if v_base > 0 and v_base < config.V_MIN_CM_S:
            v_base = config.V_MIN_CM_S
    
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
        # AJUSTADO: Límite aumentado de 0.9 → 0.95 y divisor reducido de 5.0 → 3.0
        # Esto permite que la fuerza repulsiva domine más cuando hay obstáculos cercanos
        weight_rep = min(f_rep_mag / 3.0, 0.95)
        weight_att = 1.0 - weight_rep
        
        # Combinar ángulos mediante promedio ponderado de vectores unitarios
        # Esto produce una dirección resultante que evita obstáculos mientras
        # mantiene el objetivo de avanzar hacia la meta
        combined_x = weight_att * math.cos(desired_angle_att) + weight_rep * math.cos(angle_rep)
        combined_y = weight_att * math.sin(desired_angle_att) + weight_rep * math.sin(angle_rep)
        desired_angle = math.atan2(combined_y, combined_x)
        
        # Aplicar reducción adicional de velocidad cuando hay fuerza repulsiva significativa
        # Esto proporciona una capa extra de seguridad además del límite dinámico v_max_allowed
        # AJUSTADO: Reducción menos agresiva (0.5 → 0.4) para mantener más velocidad durante evasión
        # Factor mínimo aumentado de 0.2 → 0.4 para evitar que se detenga durante la maniobra
        extra_slowdown = max(0.4, 1.0 - weight_rep * 0.4)
        v_linear = v_base * extra_slowdown
    else:
        # Sin obstáculos detectados, usar directamente el ángulo hacia la meta
        desired_angle = desired_angle_att
        v_linear = v_base
    
    # ========== CALCULAR ERROR ANGULAR Y AJUSTAR VELOCIDAD ==========
    # Calcular el error entre la dirección deseada y la orientación actual del robot
    theta_rad = math.radians(q[2])
    angle_error = _wrap_pi(desired_angle - theta_rad)
    
    # Reducir velocidad cuando el error angular es grande
    # Usamos el coseno del error como factor: valor alto cuando estamos bien orientados,
    # valor bajo cuando necesitamos girar. Mantenemos un límite inferior para permitir
    # maniobras de evasión incluso cuando el error angular es grande
    # AJUSTADO: Límite aumentado de 0.1 → 0.2 para mantener más velocidad durante giros de evasión
    # MEJORA: Cerca del objetivo (< 10cm), ser más tolerante con el ángulo para evitar oscilación
    angle_factor = math.cos(angle_error)
    
    # Ajustar límite inferior según distancia al objetivo
    if distance < 10.0:
        # Cerca del objetivo: límite más alto (0.5) para permitir avance aunque no esté perfectamente orientado
        min_angle_factor = 0.5
    else:
        # Lejos del objetivo: límite normal (0.2) para giros de evasión
        min_angle_factor = 0.2
    
    if angle_factor < min_angle_factor:
        angle_factor = min_angle_factor
    
    v_linear *= angle_factor
    
    # Asegurar velocidad mínima para evasión efectiva cuando hay obstáculos
    # Si hay fuerza repulsiva significativa pero la velocidad es muy baja (y no es
    # una situación de emergencia), establecemos una velocidad mínima baja que permite
    # al robot ejecutar maniobras de evasión
    if v_linear > 0 and v_linear < 1.0 and f_rep_mag > 1.0 and safety_level != "EMERGENCY":
        v_linear = 1.0  # Velocidad mínima baja solo para evasión
    
    # ========== CALCULAR VELOCIDAD ANGULAR ==========
    # La velocidad angular es proporcional al error angular, permitiendo corrección
    # de orientación más rápida cuando el error es mayor. Esto es especialmente
    # importante durante evasión de obstáculos cuando necesitamos cambiar dirección rápidamente
    
    # MEJORA: Reducir ganancia angular cerca del objetivo para evitar oscilación
    # Cuando estamos muy cerca (< 15cm), reducimos k_ang progresivamente para
    # permitir convergencia suave sin oscilaciones
    k_ang_adjusted = k_ang
    if distance < 15.0:
        # Reducir k_ang linealmente de 100% a 30% cuando dist va de 15cm a 5cm
        reduction_factor = 0.3 + 0.7 * ((distance - 5.0) / 10.0)
        reduction_factor = max(0.3, min(1.0, reduction_factor))
        k_ang_adjusted = k_ang * reduction_factor
    
    omega = k_ang_adjusted * angle_error
    
    # Convertir el límite de velocidad angular y saturar
    omega_max_rad_s = config.W_MAX_CM_S / (config.WHEEL_BASE_CM / 2.0)
    omega = max(-omega_max_rad_s, min(omega_max_rad_s, omega))
    
    # ========== CONVERSIÓN A VELOCIDADES DE RUEDA ==========
    # Convertir velocidad lineal y angular en velocidades individuales de rueda
    # usando la cinemática diferencial del robot
    half_base = config.WHEEL_BASE_CM / 2.0
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
        'max_ir_front': max_ir_front,
        'v_max_allowed': v_max_allowed
    }
    
    return v_left, v_right, distance, info