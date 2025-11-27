"""
Módulo de configuración centralizada para PL4 - Navegación con Campos de Potencial

===============================================================================
INFORMACIÓN DEL PROYECTO
===============================================================================

Autores:
    - Alan Ariel Salazar
    - Yago Ramos Sánchez

Institución:
    Universidad Intercontinental de la Empresa (UIE)

Profesor:
    Eladio Dapena

Asignatura:
    Robots Autónomos

Fecha de Finalización:
    6 de noviembre de 2025

Robot SDK:
    irobot-edu-sdk

===============================================================================
OBJETIVO GENERAL
===============================================================================

Centralizar todos los parámetros configurables del sistema de navegación basado
en campos de potencial, proporcionando un punto único de configuración que permita
ajustar velocidades, ganancias de control, umbrales de sensores y parámetros
específicos para cada función de potencial sin necesidad de modificar el código
principal en múltiples lugares.

===============================================================================
OBJETIVOS ESPECÍFICOS
===============================================================================

1. Agrupar todos los parámetros del sistema en secciones lógicas para facilitar
   su localización y modificación

2. Proporcionar valores calibrados experimentalmente para garantizar un
   comportamiento seguro y efectivo del robot

3. Documentar cada parámetro con su propósito, unidades y rango de valores
   recomendado

4. Facilitar la experimentación permitiendo ajustes rápidos de parámetros sin
   modificar código de lógica de control

5. Mantener consistencia de parámetros entre todos los módulos del proyecto

CONFIGURACIÓN:

Este módulo está organizado en secciones lógicas que agrupan parámetros relacionados:

- Configuración del robot: Nombre Bluetooth y geometría física
- Velocidades: Límites máximos y mínimos de movimiento
- Control: Período de muestreo y tolerancias de navegación
- Potencial atractivo: Ganancias específicas para cada función de potencial
- Potencial repulsivo: Parámetros para evasión de obstáculos
- Sensores IR: Umbrales y mapeo de ángulos calibrados experimentalmente
- Seguridad: Sistema de umbrales escalonados para control dinámico de velocidad
- Logging y teleoperación: Parámetros para herramientas auxiliares

Todos los valores están basados en calibración real del robot y pruebas extensivas
para garantizar un comportamiento seguro y efectivo.
"""

# ============ CONFIGURACIÓN DEL ROBOT ============
# Nombre Bluetooth del robot para conexión
# Este nombre debe coincidir con el configurado en el robot Create 3
BLUETOOTH_NAME = "C3_UIEC_Grupo1"

# Distancia entre las ruedas del robot en centímetros
# Este valor es crítico para convertir velocidades lineales y angulares
# en velocidades individuales de rueda
WHEEL_BASE_CM = 23.5

# ============ LÍMITES DE VELOCIDAD ============
# Velocidad máxima lineal permitida en centímetros por segundo
# Este límite protege los motores y garantiza control seguro
# REDUCIDO: 48.0 → 38.0 cm/s para dar más tiempo de reacción ante obstáculos
V_MAX_CM_S = 38.0

# Velocidad mínima antes de detención completa
# Si la velocidad calculada es menor que este valor, el robot se detiene
V_MIN_CM_S = 0.0

# Velocidad angular máxima expresada como diferencia entre ruedas
# Controla qué tan rápido puede girar el robot sin perder estabilidad
W_MAX_CM_S = 10.0

# ============ PARÁMETROS DE CONTROL ============
# Período de muestreo del bucle de control en segundos
# Este valor define la frecuencia de control (20 Hz = 50 ms)
CONTROL_DT = 0.05

# Tolerancia de distancia para considerar que se alcanzó la meta
# Si la distancia al objetivo es menor que este valor, la navegación termina
# REDUCIDO: 5.0 → 3.0 para mayor precisión en aproximación final
TOL_DIST_CM = 3.0

# Tolerancia angular en grados
# Utilizada para validaciones adicionales de orientación si es necesario
TOL_ANGLE_DEG = 5.0

# Rampa de aceleración: límite de cambio de velocidad por iteración
# Previene cambios bruscos de velocidad que podrían causar deslizamiento
# o pérdida de control. Valor en cm/s por ciclo de control
# AUMENTADO: 5.0 → 10.0 para aceleración más rápida pero suave
ACCEL_RAMP_CM_S2 = 10.0

# Velocidad inicial mínima al arrancar (cm/s)
# El robot empieza lento y acelera progresivamente
V_START_MIN_CM_S = 8.0

# Distancia de desaceleración: cuando el robot está a esta distancia del objetivo
# comienza a reducir velocidad progresivamente (cm)
# REDUCIDO: 80.0 → 50.0 para mantener más velocidad hasta más cerca
DECEL_ZONE_CM = 50.0

# Velocidad mínima en la zona de aproximación final (cm)
# REDUCIDO: 12.0 → 6.0 para aproximación más lenta y precisa
V_APPROACH_MIN_CM_S = 6.0

# ============ PARÁMETROS DE POTENCIAL ATRACTIVO ============
# Ganancia angular para corrección de orientación
# Controla qué tan agresivamente el robot corrige su dirección hacia el objetivo
# AUMENTADO: 2.0 → 3.0 para permitir reacciones de giro MÁS rápidas ante obstáculos
# Con mayor ganancia angular, el robot puede cambiar dirección más rápidamente
K_ANGULAR = 3.0

# Ganancias lineales específicas para cada tipo de función de potencial
# Cada función tiene características diferentes de escala, por lo que requiere
# ganancias ajustadas independientemente

# Función lineal: F = k * d
# Proporcional directa a la distancia
K_LINEAR = 0.25

# Función cuadrática: F = k * d²
# Requiere ganancia menor porque la distancia está al cuadrado
K_QUADRATIC = 0.05

# Función cónica: F = k * min(d, d_sat)
# Saturación a 100 cm, velocidad constante en distancias largas
K_CONIC = 0.15

# Función exponencial: F = k * (1 - e^(-d/λ))
# La salida está normalizada entre 0 y 1, requiere ganancia mayor
K_EXPONENTIAL = 2.5

# ============ PARÁMETROS DE POTENCIAL REPULSIVO ============
# Estos parámetros controlan cómo el robot evade obstáculos detectados
# por los sensores IR. Las fuerzas repulsivas ahora se calculan basándose
# en el CLEARANCE (distancia libre después del radio del robot), no solo
# en la distancia absoluta al obstáculo.

# Ganancia repulsiva que controla la intensidad de las fuerzas de evasión
# MODELO MEJORADO: Usa F = k_rep * f(clearance) donde f es no-lineal
# Valores altos generan evasión más agresiva cuando clearance < d_safe
# AUMENTADO: 150.0 → 300.0 para reacción más temprana y agresiva a obstáculos
K_REPULSIVE = 300.0

# Distancia de influencia del campo repulsivo en centímetros
# Los obstáculos más lejos que esta distancia no generan fuerzas repulsivas
# IMPORTANTE: Esta distancia se mide desde el CENTRO del robot
# AUMENTADO: 60.0 → 100.0cm para detección y reacción más temprana
D_INFLUENCE = 100.0

# Distancia de seguridad mínima (clearance mínimo recomendado)
# Esta es la distancia libre que el robot debe mantener del borde del obstáculo
# Basada en capacidad de maniobra y margen de error de sensores
# AUMENTADO: 12cm → 20cm para mayor margen de seguridad
D_SAFE = 20.0

# ============ PARÁMETROS DE ESCAPE DE TRAMPAS (Mínimos Locales) ============
# Configuración para permitir que el robot escape de situaciones de trampa en C
# donde hay obstáculos adelante, izquierda y derecha simultáneamente

# Habilitar modo de escape de trampa en C
# Cuando está atrapado, el robot reduce temporalmente la fuerza atractiva
# para permitir que las fuerzas repulsivas lo guíen hacia la apertura
ENABLE_TRAP_ESCAPE = True

# Umbral de detección de trampa: número mínimo de sensores con obstáculos
# Si 5 o más sensores detectan obstáculos simultáneamente, consideramos trampa
TRAP_DETECTION_SENSOR_COUNT = 5

# Umbral de intensidad IR para considerar un sensor como "bloqueado"
# Valores IR por encima de este umbral indican obstáculo significativo
TRAP_DETECTION_IR_THRESHOLD = 100

# Factor de reducción de fuerza atractiva cuando está atrapado
# Reducir a 30% permite que las fuerzas repulsivas dominen y encuentren salida
# Valor entre 0.0 (sin atracción) y 1.0 (atracción normal)
TRAP_ATTRACTIVE_REDUCTION = 0.3

# Aumentar fuerza repulsiva cuando está atrapado
# Multiplicador de K_REPULSIVE para hacer el esquive más agresivo
# 1.5 = +50% de fuerza repulsiva
TRAP_REPULSIVE_BOOST = 1.5

# Velocidad mínima garantizada durante escape de trampa
# Incluso en trampa, el robot debe mantener movimiento hacia adelante
# Esto previene quedarse completamente detenido (giro sobre eje)
TRAP_MIN_FORWARD_SPEED = 4.0  # cm/s

# Factor de aumento de ganancia angular durante escape
# Permite giros más cerrados para encontrar la apertura de la C
# 1.5 = +50% de capacidad de giro
TRAP_ANGULAR_BOOST = 1.5

# ============ GEOMETRÍA DEL ROBOT ============
# Dimensiones físicas del robot necesarias para cálculos de posicionamiento
# de sensores y estimación de distancias a obstáculos
# MEDIDAS REALES verificadas físicamente:

# Radio exterior del chasis en centímetros
# Medida real: diámetro = 341.9mm → radio = 170.95mm
ROBOT_RADIUS_CM = 17.095

# Diámetro efectivo del robot 
# Medida real verificada: 341.9mm = 34.19cm
ROBOT_DIAMETER_CM = 34.19

# Distancia entre centros de ruedas (wheelbase)
# Medida real verificada: 235mm = 23.5cm
# Nota: Ya está definido en WHEEL_BASE_CM = 23.5

# Diámetro de las ruedas
# Medida real verificada: 72mm = 7.2cm
WHEEL_DIAMETER_CM = 7.2

# ============ CALIBRACIÓN ESPECÍFICA POR SENSOR IR ============
# Factores de normalización basados en calibración real a 5cm
# Estos factores convierten las lecturas de cada sensor a un valor equivalente
# permitiendo usar umbrales uniformes a pesar de sensibilidades diferentes
#
# Fórmula: IR_normalizado = IR_real / SENSOR_SENSITIVITY_FACTOR
# Valores basados en mediciones perpendiculares a 5cm:

IR_SENSOR_SENSITIVITY_FACTORS = {
    0: 1382.0 / 1000.0,  # Sensor 0: 1382 a 5cm → factor 1.382
    1: 1121.0 / 1000.0,  # Sensor 1: 1121 a 5cm → factor 1.121
    2: 270.0 / 1000.0,   # Sensor 2: 270 a 5cm → factor 0.270
    3: 1045.0 / 1000.0,  # Sensor 3: 1045 a 5cm → factor 1.045
    4: 896.0 / 1000.0,   # Sensor 4: 896 a 5cm → factor 0.896
    5: 672.0 / 1000.0,   # Sensor 5: 672 a 5cm → factor 0.672
    6: 901.0 / 1000.0    # Sensor 6: 901 a 5cm → factor 0.901
}

# Valor de referencia: 1000 (valor normalizado esperado a 5cm)
IR_REFERENCE_VALUE_AT_5CM = 1000.0

# ============ CONFIGURACIÓN DE SENSORES INFRARROJOS ============
# Radio de montaje de los sensores IR asumiendo que están en el borde frontal
# del chasis del robot
IR_SENSOR_RADIUS = ROBOT_RADIUS_CM

# Mapeo de ángulos de cada sensor IR en grados desde el frente del robot
# IMPORTANTE: Usamos convención matemática estándar donde:
# - Ángulos POSITIVOS = IZQUIERDA del robot (antihorario desde el frente)
# - Ángulos NEGATIVOS = DERECHA del robot (horario desde el frente)
# 
# Nota: iRobot usa la convención opuesta en su documentación, pero aquí la
# invertimos para mantener consistencia con nuestra convención de coordenadas
# donde θ=0° apunta a +X y los ángulos crecen en sentido antihorario (atan2)
IR_SENSOR_ANGLES = {
    0: +65.3,   # Lateral IZQUIERDO extremo (físicamente a la izquierda del robot)
    1: +38.0,   # Intermedio IZQUIERDO
    2: +20.0,   # Frontal IZQUIERDO interno
    3: -3.0,    # Central (ligeramente a la derecha)
    4: -14.25,  # Frontal DERECHO interno
    5: -34.0,   # Intermedio DERECHO
    6: -65.3    # Lateral DERECHO extremo (físicamente a la derecha del robot)
}

# ============ SISTEMA DE SEGURIDAD - UMBRALES ESCALONADOS ============
# BASADO EN CALIBRACIÓN REAL MEJORADA - Distancias reales estimadas:
# Con el modelo mejorado de conversión IR→distancia:
# - IR_norm = 1000 → ~5cm (borde del robot)
# - IR_norm = 700 → ~6cm
# - IR_norm = 400 → ~9cm
# - IR_norm = 200 → ~13cm
# - IR_norm = 100 → ~18cm
# - IR_norm = 60 → ~22cm
# - IR_norm = 30 → ~35cm
#
# IMPORTANTE: Ahora los umbrales reflejan DISTANCIAS REALES después de
# compensación por ángulo del sensor. El robot puede acercarse más sin
# frenar excesivamente, mejorando eficiencia en espacios confinados.
#
# GEOMETRÍA DEL ROBOT:
# - Radio: 17.1cm → necesita mínimo 17cm de clearance
# - Con margen de seguridad: 17cm + 5cm = 22cm mínimo recomendado
# - Radio de giro mínimo: ~12cm (basado en wheelbase 23.5cm)

# Umbral de emergencia: obstáculo MUY cerca (~5-7cm del borde)
# A esta distancia solo queda ~5cm antes de contacto físico
# AUMENTADO: 400 → 700 para reflejar distancias reales
IR_THRESHOLD_EMERGENCY = 700

# Umbral crítico: obstáculo cerca (~8-11cm del borde)
# A esta distancia el robot puede girar pero debe reducir velocidad
# AUMENTADO: 150 → 350
IR_THRESHOLD_CRITICAL = 350

# Umbral de advertencia: obstáculo a distancia media (~12-17cm)
# A esta distancia el robot tiene espacio para maniobrar
# AUMENTADO: 80 → 180
IR_THRESHOLD_WARNING = 180

# Umbral de precaución: obstáculo detectado (~18-25cm)
# A esta distancia el robot debe empezar a considerar evasión
# AUMENTADO: 50 → 90
IR_THRESHOLD_CAUTION = 90

# Umbral mínimo de detección (~30-40cm)
# Valores por debajo se consideran sin obstáculo cercano
# Sin cambio: se mantiene en 30
IR_THRESHOLD_DETECT = 30

# ============ LÍMITES DINÁMICOS DE VELOCIDAD ============
# Velocidades máximas permitidas según el nivel de seguridad detectado
# MEJORADO: Velocidades aumentadas para permitir navegación eficiente
# incluso cerca de obstáculos. El robot debe ESQUIVAR ágilmente, no arrastrarse.
#
# FILOSOFÍA: Solo reducir velocidad cuando hay riesgo REAL de colisión,
# no por detectar obstáculos a distancia segura.

# Velocidad máxima en situación de emergencia (cm/s)
# Con IR >= 700 (normalizado ~1000), obstáculo a 5-7cm → Muy lento pero móvil
# AUMENTADO: 2.0 → 8.0 cm/s para mantener capacidad de maniobra
V_MAX_EMERGENCY = 8.0

# Velocidad máxima en zona crítica (cm/s)
# Con IR >= 350 (normalizado ~400-500), obstáculo a 8-11cm → Reducido pero navegable
# AUMENTADO: 4.0 → 15.0 cm/s para permitir esquives ágiles
V_MAX_CRITICAL = 15.0

# Velocidad máxima en zona de advertencia (cm/s)
# Con IR >= 180 (normalizado ~200-250), obstáculo a 12-17cm → Velocidad moderada
# AUMENTADO: 8.0 → 25.0 cm/s para navegación fluida
V_MAX_WARNING = 25.0

# Velocidad máxima en zona de precaución (cm/s)
# Con IR >= 90 (normalizado ~100-120), obstáculo a 18-25cm → Casi velocidad completa
# AUMENTADO: 15.0 → 35.0 cm/s para eficiencia en espacios confinados
V_MAX_CAUTION = 35.0

# ============ COMPATIBILIDAD CON CÓDIGO ANTERIOR ============
# Alias para mantener compatibilidad con código que usa nombres antiguos
# de umbrales (sistema binario anterior)
IR_THRESHOLD_STOP = IR_THRESHOLD_CRITICAL
IR_THRESHOLD_SLOW = IR_THRESHOLD_WARNING

# Lista de índices de sensores IR para iteración y validación
IR_INDICES = [0, 1, 2, 3, 4, 5, 6]

# ============ REFERENCIAS DE SENSORES IR ============
# Constantes con nombres descriptivos para identificar sensores específicos
# Estos valores representan los índices de los sensores en el array de lecturas
# y facilitan la referencia en el código

IR_FRONT_CENTER = 3        # Sensor central frontal, más sensible: ~1044 a 5cm
IR_FRONT_LEFT = 2          # Sensor frontal izquierdo, sensible: ~271 a 5cm
IR_FRONT_RIGHT = 4         # Sensor frontal derecho, sensible: ~895 a 5cm
IR_SIDE_LEFT = 1           # Sensor lateral izquierdo, muy sensible: ~1121 a 5cm
IR_SIDE_RIGHT = 5          # Sensor lateral derecho, moderado: ~676 a 5cm
IR_CORNER_LEFT = 0         # Sensor esquina izquierda, muy sensible: ~1382 a 5cm
IR_CORNER_RIGHT = 6        # Sensor esquina derecha, moderado: ~900 a 5cm

# ============ CONFIGURACIÓN DE LOGGING ============
# Intervalo de tiempo en segundos entre impresiones de estado de sensores
# Este valor controla la frecuencia con la que se muestra información de
# monitoreo durante la navegación
LOG_INTERVAL_S = 1.0

# ============ CONFIGURACIÓN DE TELEOPERACIÓN ============
# Parámetros para el script point_manager.py que permite control manual
# del robot para marcar puntos de navegación

# Velocidad de avance y retroceso durante teleoperación (cm/s)
TELEOP_VEL = 15

# Velocidad de giro durante teleoperación (cm/s)
# Representa la diferencia de velocidad entre ruedas para rotación
TELEOP_GIRO = 8

# ============ CONFIGURACIÓN DE ARCHIVOS ============
# Nombre del archivo JSON que contiene los puntos de navegación
# Este archivo se genera mediante point_manager.py y se lee al inicio de
# la navegación para obtener las coordenadas inicial y final
POINTS_FILE = "points.json"

# ============ CALIBRACIÓN DE CONVERSIÓN IR A DISTANCIA ============
# Datos de calibración real obtenidos con obstáculos perpendiculares
# a diferentes distancias del robot Create3

# Puntos de referencia para conversión IR→Distancia basados en calibración real:
# Formato: (distancia_cm, valor_IR_frontal_típico)
# NOTA: Los valores IR pueden variar según el ángulo y tipo de superficie
IR_CALIBRATION_POINTS = [
    (5.0, 1000),    # A 5cm del borde: ~800-1400 según sensor y ángulo
    (10.0, 400),    # Estimación intermedia basada en modelo inverso al cuadrado
    (15.0, 200),    # Estimación
    (20.0, 120),    # Estimación
    (30.0, 60),     # Umbral de precaución
    (40.0, 40),     # Umbral mínimo de detección
    (50.0, 25),     # Muy débil, casi sin obstáculo
]

# Modelo de conversión IR→Distancia
# Los sensores IR siguen aproximadamente: intensidad ∝ 1/distancia²
# Por tanto: distancia = k / sqrt(intensidad)

# Constante de calibración basada en medición a 5cm con IR≈1000
# d = k / sqrt(I)  →  5 = k / sqrt(1000)  →  k ≈ 158
IR_DISTANCE_CONSTANT = 158.0

# Rango válido de estimación de distancia
IR_MIN_DISTANCE_CM = 4.0    # Distancia mínima estimable (muy cerca)
IR_MAX_DISTANCE_CM = 60.0   # Distancia máxima estimable (señal débil)

# ============ UMBRALES ADAPTATIVOS PARA APROXIMACIÓN A WAYPOINTS ============
# Estos parámetros permiten que el robot llegue a waypoints que están cerca de
# paredes u obstáculos, reduciendo la influencia de la evasión cuando está
# muy cerca del objetivo.
#
# IMPORTANTE: Los sensores IR dan valores MÁS ALTOS cuando el obstáculo está MÁS CERCA
# Ejemplo: pared a 5cm → IR ~1000, pared a 30cm → IR ~60
#
# Caso de uso: waypoint a 5cm de una pared
# - Sin adaptación: el robot detecta pared (IR alto) y evade, nunca llega al waypoint
# - Con adaptación: cerca del waypoint, se reduce/ignora la evasión para poder llegar

# ============ PRIORIDAD DE OBJETIVO SOBRE OBSTÁCULO ============
# NUEVA FUNCIONALIDAD: Compara la distancia al nodo vs distancia al obstáculo
# Si el nodo está MÁS CERCA que el obstáculo, el robot prioriza llegar al nodo
# reduciendo o eliminando la fuerza repulsiva de ese obstáculo específico.
#
# Ejemplo: nodo a 15cm, pared a 20cm → robot ignora la pared y llega al nodo
# Ejemplo: nodo a 25cm, pared a 10cm → robot mantiene evasión normal (pared más cerca)

# Habilitar comparación distancia-al-nodo vs distancia-al-obstáculo
GOAL_PRIORITY_ENABLED = True

# Margen de seguridad en cm para la comparación
# Si distance_to_goal <= distance_to_obstacle + margen → priorizar objetivo
# Un margen pequeño (3-5cm) permite llegar a nodos cerca de paredes
# Un margen de 0 significa comparación exacta
GOAL_PRIORITY_MARGIN_CM = 5.0

# Factor de reducción de fuerza repulsiva cuando el objetivo está más cerca
# 0.0 = eliminar completamente la fuerza (más agresivo, llega más fácil)
# 0.1 = reducir al 10% (conservador, algo de evasión residual)
# 0.05 = reducir al 5% (equilibrado)
GOAL_PRIORITY_FORCE_FACTOR = 0.05

# Distancia al objetivo a partir de la cual se empieza a reducir la evasión (cm)
# Cuando el robot está más cerca que esta distancia, comienza a reducir k_rep y d_influence
APPROACH_REDUCE_START_CM = 25.0

# Distancia al objetivo en la cual la reducción es máxima (cm)
# A esta distancia y menor, los parámetros de evasión están al mínimo permitido
# IMPORTANTE: Debe ser similar o menor a la tolerancia de llegada (TOL_DIST_CM = 3cm)
APPROACH_REDUCE_END_CM = 8.0

# Factor mínimo de reducción para k_rep cuando está muy cerca del objetivo
# 0.1 significa que la fuerza repulsiva se reduce al 10% - casi ignorando obstáculos
# Esto permite llegar a waypoints que están literalmente contra paredes
APPROACH_K_REP_MIN_FACTOR = 0.1

# Factor mínimo de reducción para d_influence cuando está muy cerca del objetivo
# 0.2 significa que la distancia de influencia se reduce al 20% (de 100cm a 20cm)
# Solo obstáculos MUY cercanos generarán fuerza repulsiva
APPROACH_D_INFLUENCE_MIN_FACTOR = 0.2

# Umbral de velocidad para reducción adicional (cm/s)
# Cuando la velocidad es menor que este valor, se aplica reducción adicional
# Esto permite aproximación más precisa en velocidades bajas
APPROACH_LOW_SPEED_THRESHOLD = 12.0

# Factor adicional de reducción cuando la velocidad es baja
# Se multiplica con los otros factores para reducción aún mayor
# Con 0.5, la fuerza repulsiva puede llegar a ser solo 5% del original (0.1 * 0.5)
APPROACH_LOW_SPEED_EXTRA_FACTOR = 0.5

# ============ DETECCIÓN DE GAPS (PASILLOS) NAVEGABLES ============
# Parámetros para detectar espacios entre obstáculos por donde el robot puede pasar

# Ancho mínimo del gap para considerar navegable (cm)
# Debe ser mayor que el diámetro del robot (34.19cm) más un margen de seguridad
# Con diámetro real de 34.19cm, necesitamos:
# - Diámetro robot: 34.19cm
# - Margen seguridad: ~15cm por lado (tolerancia de control y giros)
# - Total mínimo: 34.19 + 30 = ~65cm
GAP_MIN_WIDTH_CM = 65.0

# Diferencia mínima de ángulo entre sensores adyacentes para considerar como gap (grados)
# Los sensores están típicamente separados por 15-30 grados
GAP_MIN_ANGLE_SEPARATION_DEG = 15.0

# Umbral de IR para considerar un sensor como "libre" (sin obstáculo significativo)
# Valores menores que este umbral indican espacio libre
GAP_CLEAR_THRESHOLD = 60

# Umbral de IR para considerar un sensor como "bloqueado" (con obstáculo)
# Valores mayores que este umbral indican obstáculo presente
GAP_BLOCKED_THRESHOLD = 100

# Factor de reducción de fuerza repulsiva cuando hay gap navegable
# Reduce las fuerzas repulsivas de los obstáculos laterales del gap
# para permitir que el robot pase entre ellos
GAP_REPULSION_REDUCTION_FACTOR = 0.3  # Reducir al 30%

# ============ GEOMETRÍA DE NAVEGACIÓN POR GAPS ============
# Radio de seguridad efectivo considerando el ancho del robot y margen de maniobra
# Usado para calcular si el robot cabe en un gap detectado
ROBOT_WIDTH_WITH_MARGIN_CM = ROBOT_DIAMETER_CM + 25.0  # 34.19 + 25 = ~59.19 cm
