# core/config_validator.py
# Validador de configuración basado en valores probados de PL2/examples
# Extrae valores exactos de los laboratorios y ejemplos funcionando

import yaml
import os
from typing import Dict, Any, Tuple

# Valores de referencia extraídos de PL2/examples (PROBADOS)
REFERENCE_VALUES = {
    # PL2/T02_Etapa01.py, T02_Etapa02.py, etc.
    "ir_obs_threshold": 120,      # ~15 cm (CRÍTICO: no modificar)
    "ir_dir_threshold": 200,      # Umbral para decisión de giro (CRÍTICO: no cambiar)
    
    # examples/manual_move.py
    "vel_advance": 15,            # Avance/retroceso (cm/s)
    "vel_giro": 10,               # Diferencia entre ruedas para girar (cm/s)
    "track_width_cm": 23.5,       # Geometría del robot (distancia entre ruedas)
    
    # examples/ir_proximity_obstacles.py
    "ir_threshold_obstacles": 150, # Umbral para obstáculos
    
    # PL2 - velocidades de navegación
    "vel_navigation": 5,          # Velocidad de navegación en PL2 (cm/s)
    
    # Geometría y cálculos
    "deg_por_seg": None,          # Se calcula: (2.0 * GIRO / ANCHO_EJE_CM) * (180.0 / math.pi)
}

def calculate_deg_per_seg(giro_cm_s: float, track_width_cm: float) -> float:
    """Calcula grados por segundo basado en geometría del robot"""
    import math
    return (2.0 * giro_cm_s / track_width_cm) * (180.0 / math.pi)

def validate_config(config_path: str = "config.yaml") -> Tuple[bool, Dict[str, Any], str]:
    """
    Valida config.yaml contra valores de referencia probados.
    
    Returns:
        (is_valid, config_dict, error_message)
    """
    if not os.path.exists(config_path):
        return False, {}, f"Archivo {config_path} no encontrado"
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f) or {}
    except Exception as e:
        return False, {}, f"Error leyendo {config_path}: {e}"
    
    errors = []
    
    # Validar estructura básica
    required_sections = ['robot', 'motion', 'safety', 'undock', 'telemetry']
    for section in required_sections:
        if section not in config:
            errors.append(f"Sección '{section}' faltante en config.yaml")
    
    if errors:
        return False, config, "; ".join(errors)
    
    # Validar robot
    robot_config = config.get('robot', {})
    if 'name' not in robot_config:
        errors.append("robot.name faltante")
    
    # Validar motion (valores de examples/manual_move.py)
    motion_config = config.get('motion', {})
    expected_motion = {
        'vel_default_cm_s': (15, 25),      # Rango basado en examples
        'giro_default_cm_s': (8, 15),      # Rango basado en examples  
        'track_width_cm': (23.0, 24.0),    # Valor exacto de examples
        'linear_scale': (0.5, 1.5),        # Factor de corrección lineal
        'angular_scale': (0.5, 1.5),       # Factor de corrección angular
    }
    
    for key, (min_val, max_val) in expected_motion.items():
        if key not in motion_config:
            errors.append(f"motion.{key} faltante")
        else:
            try:
                val = float(motion_config[key])
                if not (min_val <= val <= max_val):
                    errors.append(f"motion.{key}={val} fuera de rango [{min_val}, {max_val}]")
            except (ValueError, TypeError):
                errors.append(f"motion.{key} debe ser numérico")
    
    # Validar safety (valores de PL2)
    safety_config = config.get('safety', {})
    expected_safety = {
        'ir_threshold': (100, 200),        # Rango basado en PL2 (120) y examples (150)
        'safety_period_s': (0.05, 0.5),    # Rango razonable
        'enable_auto_brake': bool,          # Debe ser boolean
    }
    
    for key, expected_type in expected_safety.items():
        if key not in safety_config:
            errors.append(f"safety.{key} faltante")
        else:
            if key == 'enable_auto_brake':
                if not isinstance(safety_config[key], bool):
                    errors.append(f"safety.{key} debe ser boolean")
            else:
                try:
                    val = float(safety_config[key])
                    if isinstance(expected_type, tuple):
                        min_val, max_val = expected_type
                        if not (min_val <= val <= max_val):
                            errors.append(f"safety.{key}={val} fuera de rango [{min_val}, {max_val}]")
                except (ValueError, TypeError):
                    errors.append(f"safety.{key} debe ser numérico")
    
    # Validar undock
    undock_config = config.get('undock', {})
    expected_undock = {
        'back_cm': (10, 50),               # Rango razonable
        'back_speed': (3, 10),             # Rango basado en PL2 (5 cm/s)
        'turn_deg': (0, 360),              # Cualquier ángulo
        'turn_dir': ['left', 'right'],     # Valores válidos
    }
    
    for key, expected in expected_undock.items():
        if key not in undock_config:
            errors.append(f"undock.{key} faltante")
        else:
            if key == 'turn_dir':
                if undock_config[key] not in expected:
                    errors.append(f"undock.{key} debe ser uno de {expected}")
            else:
                try:
                    val = float(undock_config[key])
                    if isinstance(expected, tuple):
                        min_val, max_val = expected
                        if not (min_val <= val <= max_val):
                            errors.append(f"undock.{key}={val} fuera de rango [{min_val}, {max_val}]")
                except (ValueError, TypeError):
                    errors.append(f"undock.{key} debe ser numérico")
    
    # Validar telemetry
    telemetry_config = config.get('telemetry', {})
    if 'period_s' not in telemetry_config:
        errors.append("telemetry.period_s faltante")
    else:
        try:
            period = float(telemetry_config.get('period_s'))
            if not (0.05 <= period <= 1.0):
                errors.append(f"telemetry.period_s={period} fuera de rango [0.05, 1.0]")
        except (ValueError, TypeError):
            errors.append("telemetry.period_s debe ser numérico")
    if 'log_dir' not in telemetry_config or not isinstance(telemetry_config.get('log_dir'), str):
        errors.append("telemetry.log_dir faltante o inválido")
    
    if errors:
        return False, config, "; ".join(errors)
    
    # Validar potential_nav (opcional pero recomendado)
    pnav = config.get('potential_nav', {})
    if pnav:
        try:
            float(pnav.get('k_linear', 0.25))
            float(pnav.get('k_quadratic', 0.05))
            float(pnav.get('k_conic', 0.15))
            float(pnav.get('k_exponential', 2.5))
            float(pnav.get('k_angular', 3.0))
            float(pnav.get('k_repulsive', 300.0))
            float(pnav.get('d_influence_cm', 100.0))
            float(pnav.get('v_max_cm_s', 38.0))
            float(pnav.get('tolerance_cm', 10.0))
            float(pnav.get('control_dt', 0.05))
            int(pnav.get('ir_threshold_caution', 90))
            int(pnav.get('ir_threshold_warning', 180))
            default_type = pnav.get('default_type', 'linear')
            if default_type not in ['linear', 'quadratic', 'conic', 'exponential']:
                errors.append("potential_nav.default_type debe ser linear/quadratic/conic/exponential")
        except Exception:
            errors.append("potential_nav contiene valores no numéricos")
    
    return True, config, ""

def get_validated_config(config_path: str = "config.yaml") -> Dict[str, Any]:
    """
    Obtiene configuración validada o falla con mensaje claro.
    """
    is_valid, config, error = validate_config(config_path)
    if not is_valid:
        raise ValueError(f"Configuración inválida: {error}")
    return config

def print_config_summary(config: Dict[str, Any]) -> None:
    """Imprime resumen de configuración activa para diagnóstico"""
    print("=== CONFIGURACIÓN ACTIVA ===")
    
    # Robot
    robot = config.get('robot', {})
    print(f"Robot: {robot.get('name', 'NO DEFINIDO')}")
    
    # Motion
    motion = config.get('motion', {})
    vel = motion.get('vel_default_cm_s', 'NO DEFINIDO')
    giro = motion.get('giro_default_cm_s', 'NO DEFINIDO')
    track = motion.get('track_width_cm', 'NO DEFINIDO')
    lscale = motion.get('linear_scale', 1.0)
    ascale = motion.get('angular_scale', 1.0)
    print(f"Motion: vel={vel} cm/s, giro={giro} cm/s, track={track} cm, scales(lin={lscale}, ang={ascale})")
    
    # Safety
    safety = config.get('safety', {})
    ir_thresh = safety.get('ir_threshold', 'NO DEFINIDO')
    enabled = safety.get('enable_auto_brake', 'NO DEFINIDO')
    print(f"Safety: ir_threshold={ir_thresh}, enabled={enabled}")
    
    # Undock
    undock = config.get('undock', {})
    back = undock.get('back_cm', 'NO DEFINIDO')
    turn = undock.get('turn_deg', 'NO DEFINIDO')
    dir_turn = undock.get('turn_dir', 'NO DEFINIDO')
    print(f"Undock: back={back} cm, turn={turn}°, dir={dir_turn}")
    
    # Telemetry
    telemetry = config.get('telemetry', {})
    period = telemetry.get('period_s', 'NO DEFINIDO')
    logdir = telemetry.get('log_dir', 'NO DEFINIDO')
    print(f"Telemetry: period={period}s, dir={logdir}")
    
    # Potential Nav
    pnav = config.get('potential_nav', {})
    if pnav:
        print("PotentialNav: enabled (params present)")
    
    print("=" * 30)

if __name__ == "__main__":
    # Test del validador
    try:
        config = get_validated_config()
        print_config_summary(config)
        print("✓ Configuración válida")
    except ValueError as e:
        print(f"❌ {e}")
