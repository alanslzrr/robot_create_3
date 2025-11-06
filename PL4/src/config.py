"""
Módulo de configuración centralizada para PL4 - Navegación con Campos de Potencial

Autores: Alan Salazar, Yago Ramos
Fecha: 4 de noviembre de 2025
Institución: UIE Universidad Intercontinental de la Empresa
Asignatura: Robots Autónomos - Profesor Eladio Dapena
Robot SDK: irobot-edu-sdk

OBJETIVOS PRINCIPALES:

En este módulo centralizamos todos los parámetros configurables del sistema de
navegación basado en campos de potencial. El objetivo principal era crear un
punto único de configuración que permitiera ajustar velocidades, ganancias de
control, umbrales de sensores y parámetros específicos para cada función de
potencial sin necesidad de modificar el código principal en múltiples lugares.

Esta centralización nos permite realizar calibraciones experimentales de manera
eficiente, ajustando constantes en un solo lugar y asegurando consistencia de
parámetros entre todos los módulos del proyecto. Los valores aquí definidos han
sido ajustados mediante pruebas experimentales con el robot Create 3 para lograr
un comportamiento óptimo en diferentes condiciones de navegación.

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
V_MAX_CM_S = 48.0

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
TOL_DIST_CM = 5.0

# Tolerancia angular en grados
# Utilizada para validaciones adicionales de orientación si es necesario
TOL_ANGLE_DEG = 5.0

# Rampa de aceleración: límite de cambio de velocidad por iteración
# Previene cambios bruscos de velocidad que podrían causar deslizamiento
# o pérdida de control. Valor en cm/s²
ACCEL_RAMP_CM_S2 = 5.0

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
# Estos parámetros están ajustados para permitir EVASIÓN AGRESIVA de obstáculos
# Las fuerzas deben ser lo suficientemente FUERTES para desviar la trayectoria
# ANTES de que el robot colisione, pero NO TAN FUERTES que dominen completamente

# Ganancia repulsiva que controla la intensidad de las fuerzas de evasión
# Valores altos generan evasión más agresiva, valores bajos permiten acercarse más
# NUEVA ESTRATEGIA: En lugar de empujar LEJOS, guiamos HACIA espacio libre
# Por tanto necesitamos valor MUY REDUCIDO que solo CORRIGE dirección, no domina
# REDUCIDO: 4000 → 500 para enfoque direccional inteligente
K_REPULSIVE = 500.0

# Distancia de influencia del campo repulsivo en centímetros
# Los obstáculos más lejos que esta distancia no generan fuerzas repulsivas
# IMPORTANTE: Esta distancia se mide desde el BORDE del robot, no desde el centro
# REDUCIDO: 100cm → 80cm para reducir influencia de obstáculos lejanos
# Con diámetro del robot (34.19cm) esto da margen razonable
D_INFLUENCE = 80.0

# Distancia de seguridad mínima absoluta
# Referencia para validaciones adicionales si es necesario
# AUMENTADO: 8.0 → 12.0cm para compensar imprecisiones de estimación
D_SAFE = 12.0

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
# BASADO EN CALIBRACIÓN REAL - Distancia de 5cm del borde:
# - Sensor 0 (lateral izq extremo): 1382-1386 a 5cm - MUY SENSIBLE
# - Sensor 1 (lateral izq): 1121-1123 a 5cm - MUY SENSIBLE
# - Sensor 2 (frontal izq): 268-271 a 5cm - MODERADO
# - Sensor 3 (frontal centro): 1044-1046 a 5cm - MUY SENSIBLE
# - Sensor 4 (frontal der): 895-898 a 5cm - SENSIBLE
# - Sensor 5 (lateral der): 669-676 a 5cm - MENOS SENSIBLE
# - Sensor 6 (lateral der extremo): 900-902 a 5cm - MODERADO
#
# IMPORTANTE: Los sensores tienen DIFERENTE sensibilidad
# Los frontales (0,1,3,4) son más sensibles que los laterales (5,6)
# Ajustamos umbrales para compensar estas diferencias
#
# ESTRATEGIA: Umbrales más altos = obstáculo MÁS cerca
# Con calibración real, definimos umbrales conservadores

# Umbral de emergencia: obstáculo MUY cerca (<5cm)
# Basado en valores mínimos a 5cm: sensor 2 da ~270, otros >600
# Usamos promedio: (270+600)/2 = ~400 para balance
IR_THRESHOLD_EMERGENCY = 400

# Umbral crítico: obstáculo cerca (~8-12cm estimado)
# A esta distancia el IR cae a ~150-250 según sensor
IR_THRESHOLD_CRITICAL = 150

# Umbral de advertencia: obstáculo a distancia media (~15-25cm)
# A esta distancia IR ~80-120
IR_THRESHOLD_WARNING = 80

# Umbral de precaución: obstáculo detectado (~30-50cm)
# A esta distancia IR ~40-60
IR_THRESHOLD_CAUTION = 50

# Umbral mínimo de detección
# Valores por debajo se consideran sin obstáculo
IR_THRESHOLD_DETECT = 30

# ============ LÍMITES DINÁMICOS DE VELOCIDAD ============
# Velocidades máximas permitidas según el nivel de seguridad detectado
# CRÍTICO: Las velocidades deben permitir MOVIMIENTO CONTINUO incluso con obstáculos
# La idea es ESQUIVAR, no DETENERSE

# Velocidad máxima en situación de emergencia (cm/s)
# Con IR >= 400, el obstáculo está a <5cm → Muy lento pero SIN DETENERSE
# AUMENTADO: 0.5 → 2.0 cm/s para mantener movimiento
V_MAX_EMERGENCY = 2.0

# Velocidad máxima en zona crítica (cm/s)
# Con IR >= 150, obstáculo a ~8-12cm → Lento pero navegable
# AUMENTADO: 0.8 → 4.0 cm/s
V_MAX_CRITICAL = 4.0

# Velocidad máxima en zona de advertencia (cm/s)
# Con IR >= 80, obstáculo a ~15-25cm → Velocidad reducida
# AUMENTADO: 1.5 → 8.0 cm/s
V_MAX_WARNING = 8.0

# Velocidad máxima en zona de precaución (cm/s)
# Con IR >= 50, obstáculo a ~30-50cm → Velocidad moderada
# AUMENTADO: 4.0 → 15.0 cm/s para permitir esquivar fluidamente
V_MAX_CAUTION = 15.0

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
