"""
Configuración centralizada para PL4 - Navegación con Potencial Atractivo

Autores: Yago Ramos - Salazar Alan
Fecha de finalización: 28 de octubre de 2025
Institución: UIE - Robots Autónomos
Robot SDK: irobot-edu-sdk

Objetivo:
    Centralizar todos los parámetros configurables del sistema de navegación
    basado en campos de potencial. Permite ajustar velocidades, ganancias de
    control, umbrales de sensores y parámetros específicos para cada función
    de potencial sin modificar el código principal.

Comportamiento esperado:
    - Proporcionar valores por defecto optimizados para el iRobot Create 3
    - Facilitar la calibración experimental mediante ajuste de constantes
    - Asegurar consistencia de parámetros entre todos los módulos del proyecto

Parámetros principales:
    - BLUETOOTH_NAME: Identificador del robot para conexión
    - V_MAX_CM_S: Velocidad lineal máxima permitida (48 cm/s)
    - V_MIN_CM_S: Velocidad mínima antes de detención completa (0 cm/s)
    - CONTROL_DT: Período de muestreo del bucle de control (50 ms)
    - K_LINEAR, K_QUADRATIC, K_CONIC, K_EXPONENTIAL: Ganancias específicas
      ajustadas a la escala de salida de cada función de potencial
    - IR_THRESHOLD_STOP/SLOW: Umbrales de sensores IR calibrados experimentalmente
    - ACCEL_RAMP_CM_S2: Límite de aceleración para arranque suave (5 cm/s²)
"""

# ============ ROBOT ============
BLUETOOTH_NAME = "C3_UIEC_Grupo1"
WHEEL_BASE_CM = 23.5  # Distancia entre ruedas

# ============ VELOCIDADES ============
V_MAX_CM_S = 48.0     # Velocidad máxima lineal (cm/s)
V_MIN_CM_S = 0.0      # Velocidad mínima para avanzar
W_MAX_CM_S = 10.0     # Velocidad angular máxima (diferencia entre ruedas)

# ============ CONTROL ============
CONTROL_DT = 0.05     # Periodo de control (s) - 20 Hz
TOL_DIST_CM = 3.0     # Tolerancia de llegada (cm)
TOL_ANGLE_DEG = 5.0   # Tolerancia angular (grados)

# Rampa de aceleración
ACCEL_RAMP_CM_S2 = 5.0  # Aceleración máxima (cm/s²)

# ============ POTENCIAL ATRACTIVO ============
# Ganancias por tipo de potencial (ajustadas a sus escalas)
K_ANGULAR = 1.2       # Ganancia angular (aumentada para giros más rápidos al evadir)

# Ganancias lineales específicas por función
K_LINEAR = 0.25       # Linear: F = k*d
K_QUADRATIC = 0.01    # Quadratic: F = k*d² (necesita k menor por d²)
K_CONIC = 0.15        # Conic: F = k*min(d, d_sat) (saturación en 100cm)
K_EXPONENTIAL = 2.5   # Exponential: F = k*(1-e^(-d/λ)) (salida 0-1)

# ============ POTENCIAL REPULSIVO ============
# Ajustado para EVADIR (no detener): fuerzas moderadas que desvían trayectoria
K_REPULSIVE = 500.0       # Ganancia repulsiva ALTA
D_INFLUENCE = 30.0        # Distancia de influencia repulsiva (cm)
D_SAFE = 8.0              # Distancia de seguridad mínima (cm)

# Geometría del robot
ROBOT_RADIUS_CM = 16.2    # Radio exterior del chasis (162.0 mm)
ROBOT_DIAMETER_CM = 29.42 # Diámetro efectivo del robot (294.2 mm)

# Mapeo de sensores IR a ángulos y posiciones relativas
# Radio de montaje de sensores (asumiendo montaje en borde frontal)
IR_SENSOR_RADIUS = ROBOT_RADIUS_CM

# Ángulos de cada sensor (en grados desde el frente del robot)
# IMPORTANTE: Usamos convención matemática estándar donde:
# - Ángulos POSITIVOS = IZQUIERDA del robot (antihorario desde frente)
# - Ángulos NEGATIVOS = DERECHA del robot (horario desde frente)
# Nota: iRobot usa la convención opuesta en sus docs, aquí la invertimos para
# consistencia con θ=0°→+X, antihorario positivo (atan2)
IR_SENSOR_ANGLES = {
    0: +65.3,   # Lateral IZQUIERDO extremo (físicamente a la izq del robot)
    1: +38.0,   # Intermedio IZQUIERDO
    2: +20.0,   # Frontal IZQUIERDO interno
    3: -3.0,    # Central (ligeramente a la derecha)
    4: -14.25,  # Frontal DERECHO interno
    5: -34.0,   # Intermedio DERECHO
    6: -65.3    # Lateral DERECHO extremo (físicamente a la der del robot)
}

# ============ SEGURIDAD - SENSORES IR ============
# Basado en calibración real a 5cm:
# - Frontales directos (3,4): ~900-1050
# - Frontales esquina (0,1,2): ~270-1380
# - Laterales (5,6): ~660-900
#
# Sistema de umbrales escalonados para control robusto:

IR_THRESHOLD_EMERGENCY = 800    # PARADA DE EMERGENCIA (obstáculo <5cm perpendicular)
IR_THRESHOLD_CRITICAL = 400     # CRÍTICO: reducir a 20% de velocidad
IR_THRESHOLD_WARNING = 200      # ADVERTENCIA: reducir a 50% de velocidad  
IR_THRESHOLD_CAUTION = 100      # PRECAUCIÓN: reducir a 70% de velocidad
IR_THRESHOLD_DETECT = 50        # Umbral mínimo de detección

# Límites dinámicos de velocidad según proximidad
V_MAX_EMERGENCY = 0.0           # Parar completamente
V_MAX_CRITICAL = 10.0           # Velocidad máxima en zona crítica (cm/s)
V_MAX_WARNING = 20.0            # Velocidad máxima en zona de advertencia (cm/s)
V_MAX_CAUTION = 30.0            # Velocidad máxima en zona de precaución (cm/s)

# Compatibilidad con código antiguo
IR_THRESHOLD_STOP = IR_THRESHOLD_CRITICAL   # Alias
IR_THRESHOLD_SLOW = IR_THRESHOLD_WARNING    # Alias

IR_INDICES = [0, 1, 2, 3, 4, 5, 6]

# Configuración de sensores
IR_FRONT_CENTER = 3        # Más sensible: ~1044
IR_FRONT_LEFT = 2          # Sensible: ~271
IR_FRONT_RIGHT = 4         # Sensible: ~895
IR_SIDE_LEFT = 1           # Muy sensible: ~1121
IR_SIDE_RIGHT = 5          # Moderado: ~676
IR_CORNER_LEFT = 0         # Muy sensible: ~1382
IR_CORNER_RIGHT = 6        # Moderado: ~900

# ============ LOGGING ============
LOG_INTERVAL_S = 1.0  # Intervalo de impresión de sensores

# ============ TELEOPERACIÓN ============
TELEOP_VEL = 15       # Velocidad de teleoperación (cm/s)
TELEOP_GIRO = 8       # Velocidad de giro en teleoperación

# ============ ARCHIVOS ============
POINTS_FILE = "points.json"
