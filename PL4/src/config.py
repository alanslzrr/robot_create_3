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
TOL_DIST_CM = 3.0

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
# Valor aumentado para permitir giros más rápidos durante evasión de obstáculos
K_ANGULAR = 1.2

# Ganancias lineales específicas para cada tipo de función de potencial
# Cada función tiene características diferentes de escala, por lo que requiere
# ganancias ajustadas independientemente

# Función lineal: F = k * d
# Proporcional directa a la distancia
K_LINEAR = 0.25

# Función cuadrática: F = k * d²
# Requiere ganancia menor porque la distancia está al cuadrado
K_QUADRATIC = 0.01

# Función cónica: F = k * min(d, d_sat)
# Saturación a 100 cm, velocidad constante en distancias largas
K_CONIC = 0.15

# Función exponencial: F = k * (1 - e^(-d/λ))
# La salida está normalizada entre 0 y 1, requiere ganancia mayor
K_EXPONENTIAL = 2.5

# ============ PARÁMETROS DE POTENCIAL REPULSIVO ============
# Estos parámetros están ajustados para permitir EVASIÓN efectiva de obstáculos
# sin detener completamente el robot. Las fuerzas son moderadas para desviar
# la trayectoria en lugar de causar paradas bruscas

# Ganancia repulsiva que controla la intensidad de las fuerzas de evasión
# Valores altos generan evasión más agresiva, valores bajos permiten acercarse más
K_REPULSIVE = 500.0

# Distancia de influencia del campo repulsivo en centímetros
# Los obstáculos más lejos que esta distancia no generan fuerzas repulsivas
D_INFLUENCE = 30.0

# Distancia de seguridad mínima absoluta
# Referencia para validaciones adicionales si es necesario
D_SAFE = 8.0

# ============ GEOMETRÍA DEL ROBOT ============
# Dimensiones físicas del robot necesarias para cálculos de posicionamiento
# de sensores y estimación de distancias a obstáculos

# Radio exterior del chasis en centímetros (162.0 mm según especificaciones)
ROBOT_RADIUS_CM = 16.2

# Diámetro efectivo del robot considerando el espacio necesario para movimiento
ROBOT_DIAMETER_CM = 29.42

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
# Estos umbrales están basados en calibración real realizada con obstáculos
# a 5 cm del robot. Los valores de sensores IR varían según:
# - Frontales directos (sensores 3,4): ~900-1050
# - Frontales en esquina (sensores 0,1,2): ~270-1380
# - Laterales (sensores 5,6): ~660-900
#
# Implementamos un sistema de umbrales escalonados para control robusto que
# reduce gradualmente la velocidad según la proximidad de obstáculos detectados,
# garantizando tiempo suficiente de reacción

# Umbral de emergencia: obstáculo muy cerca (<5cm perpendicular)
# En este caso el robot debe detenerse completamente
IR_THRESHOLD_EMERGENCY = 800

# Umbral crítico: obstáculo cerca (~5-10cm)
# Velocidad reducida a máximo 10 cm/s
IR_THRESHOLD_CRITICAL = 400

# Umbral de advertencia: obstáculo a distancia media (~10-20cm)
# Velocidad reducida a máximo 20 cm/s
IR_THRESHOLD_WARNING = 200

# Umbral de precaución: obstáculo lejano (~20-40cm)
# Velocidad reducida a máximo 30 cm/s
IR_THRESHOLD_CAUTION = 100

# Umbral mínimo de detección
# Valores por debajo de este umbral se consideran sin obstáculo
IR_THRESHOLD_DETECT = 50

# ============ LÍMITES DINÁMICOS DE VELOCIDAD ============
# Velocidades máximas permitidas según el nivel de seguridad detectado
# Estos límites se aplican dinámicamente durante la navegación para garantizar
# tiempo suficiente de frenado ante obstáculos

# Velocidad máxima en situación de emergencia: parar completamente
V_MAX_EMERGENCY = 0.0

# Velocidad máxima en zona crítica (cm/s)
V_MAX_CRITICAL = 10.0

# Velocidad máxima en zona de advertencia (cm/s)
V_MAX_WARNING = 20.0

# Velocidad máxima en zona de precaución (cm/s)
V_MAX_CAUTION = 30.0

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
