"""
Configuración completa de navegación por campos de potencial.
Basada en PL4/src/config.py con opción de sobrescribir parámetros desde
Proyecto_Final/config.yaml (sección potential_nav).
"""

import os
import yaml

# Defaults copiados de PL4/src/config.py

BLUETOOTH_NAME = "C3_UIEC_Grupo1"
WHEEL_BASE_CM = 23.5
V_MAX_CM_S = 38.0
V_MIN_CM_S = 0.0
W_MAX_CM_S = 10.0
CONTROL_DT = 0.05
TOL_DIST_CM = 5.0
TOL_ANGLE_DEG = 5.0
ACCEL_RAMP_CM_S2 = 10.0
V_START_MIN_CM_S = 8.0
DECEL_ZONE_CM = 80.0
V_APPROACH_MIN_CM_S = 12.0
K_ANGULAR = 3.0
K_LINEAR = 0.25
K_QUADRATIC = 0.05
K_CONIC = 0.15
K_EXPONENTIAL = 2.5
K_REPULSIVE = 300.0
D_INFLUENCE = 100.0
D_SAFE = 20.0
ENABLE_TRAP_ESCAPE = True
TRAP_DETECTION_SENSOR_COUNT = 5
TRAP_DETECTION_IR_THRESHOLD = 100
TRAP_ATTRACTIVE_REDUCTION = 0.3
TRAP_REPULSIVE_BOOST = 1.5
TRAP_MIN_FORWARD_SPEED = 4.0
TRAP_ANGULAR_BOOST = 1.5
ROBOT_RADIUS_CM = 17.095
ROBOT_DIAMETER_CM = 34.19
WHEEL_DIAMETER_CM = 7.2
IR_SENSOR_SENSITIVITY_FACTORS = {
    0: 1382.0 / 1000.0,
    1: 1121.0 / 1000.0,
    2: 270.0 / 1000.0,
    3: 1045.0 / 1000.0,
    4: 896.0 / 1000.0,
    5: 672.0 / 1000.0,
    6: 901.0 / 1000.0,
}
IR_REFERENCE_VALUE_AT_5CM = 1000.0
IR_SENSOR_RADIUS = ROBOT_RADIUS_CM
IR_SENSOR_ANGLES = {
    0: +65.3,
    1: +38.0,
    2: +20.0,
    3: -3.0,
    4: -14.25,
    5: -34.0,
    6: -65.3,
}
IR_THRESHOLD_EMERGENCY = 700
IR_THRESHOLD_CRITICAL = 350
IR_THRESHOLD_WARNING = 180
IR_THRESHOLD_CAUTION = 90
IR_THRESHOLD_DETECT = 30
V_MAX_EMERGENCY = 8.0
V_MAX_CRITICAL = 15.0
V_MAX_WARNING = 25.0
V_MAX_CAUTION = 35.0
IR_THRESHOLD_STOP = IR_THRESHOLD_CRITICAL
IR_THRESHOLD_SLOW = IR_THRESHOLD_WARNING
IR_INDICES = [0, 1, 2, 3, 4, 5, 6]
IR_FRONT_CENTER = 3
IR_FRONT_LEFT = 2
IR_FRONT_RIGHT = 4
IR_SIDE_LEFT = 1
IR_SIDE_RIGHT = 5
IR_CORNER_LEFT = 0
IR_CORNER_RIGHT = 6
LOG_INTERVAL_S = 1.0
TELEOP_VEL = 15
TELEOP_GIRO = 8
POINTS_FILE = "points.json"
IR_CALIBRATION_POINTS = [
    (5.0, 1000),
    (10.0, 400),
    (15.0, 200),
    (20.0, 120),
    (30.0, 60),
    (40.0, 40),
    (50.0, 25),
]
IR_DISTANCE_CONSTANT = 158.0
IR_MIN_DISTANCE_CM = 4.0
IR_MAX_DISTANCE_CM = 60.0
GAP_MIN_WIDTH_CM = 65.0
GAP_MIN_ANGLE_SEPARATION_DEG = 15.0
GAP_CLEAR_THRESHOLD = 60
GAP_BLOCKED_THRESHOLD = 100
GAP_REPULSION_REDUCTION_FACTOR = 0.3
ROBOT_WIDTH_WITH_MARGIN_CM = ROBOT_DIAMETER_CM + 25.0

# Cargar overrides desde config.yaml/potential_nav si existe
_CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config.yaml")

def _load_overrides():
    if not os.path.exists(_CONFIG_PATH):
        return {}
    with open(_CONFIG_PATH, "r", encoding="utf-8") as f:
        return (yaml.safe_load(f) or {}).get("potential_nav", {}) or {}

_OVERRIDES = _load_overrides()

def _apply_override(name, default):
    if name in _OVERRIDES:
        return type(default)(_OVERRIDES[name])
    return default

K_LINEAR = _apply_override("k_linear", K_LINEAR)
K_QUADRATIC = _apply_override("k_quadratic", K_QUADRATIC)
K_CONIC = _apply_override("k_conic", K_CONIC)
K_EXPONENTIAL = _apply_override("k_exponential", K_EXPONENTIAL)
K_ANGULAR = _apply_override("k_angular", K_ANGULAR)
K_REPULSIVE = _apply_override("k_repulsive", K_REPULSIVE)
D_INFLUENCE = _apply_override("d_influence_cm", D_INFLUENCE)
V_MAX_CM_S = _apply_override("v_max_cm_s", V_MAX_CM_S)
TOL_DIST_CM = _apply_override("tolerance_cm", TOL_DIST_CM)
CONTROL_DT = _apply_override("control_dt", CONTROL_DT)
IR_THRESHOLD_CAUTION = _apply_override("ir_threshold_caution", IR_THRESHOLD_CAUTION)
IR_THRESHOLD_WARNING = _apply_override("ir_threshold_warning", IR_THRESHOLD_WARNING)


