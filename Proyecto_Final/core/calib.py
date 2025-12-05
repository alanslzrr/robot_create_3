# core/calib.py
# Rutinas de calibración para el Create3
# - Track width (ancho de eje): giro 360° y comparar error
# - Escala lineal: avance largo y comparar distancia

import math
import yaml
import os

CONFIG_FILE = "config.yaml"

def load_config():
    """Carga configuración desde config.yaml"""
    if not os.path.exists(CONFIG_FILE):
        # Estructura mínima alineada con el resto del sistema
        return {
            "motion": {
                "track_width_cm": 23.5,
                "vel_default_cm_s": 20.0,
                "giro_default_cm_s": 10.0,
                "linear_scale": 1.0,
                "angular_scale": 1.0,
            }
        }
    
    with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def save_config(config):
    """Guarda configuración en config.yaml"""
    with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True)

async def calibrate_turn_360(robot):
    """
    Calibra el track width mediante giro de 360°
    Retorna el factor de corrección
    """
    print("Calibración de giro 360°")
    print("El robot girará 360° (4x90°) y mediremos el error...")
    
    # Resetear navegación
    await robot.reset_navigation()
    
    # Leer pose inicial
    p_start = await robot.get_position()
    try:
        theta_start = p_start.heading
    except AttributeError:
        theta_start = p_start[2]
    
    print(f"Orientación inicial: {theta_start:.2f}°")
    
    # Girar 4 veces 90° = 360°
    for i in range(4):
        await robot.turn_right(90)
        await robot.wait(0.5)
    
    # Leer pose final
    p_end = await robot.get_position()
    try:
        theta_end = p_end.heading
    except AttributeError:
        theta_end = p_end[2]
    
    print(f"Orientación final: {theta_end:.2f}°")
    
    # Calcular error (debería ser ~0 tras 360°)
    error = abs(theta_end - theta_start)
    if error > 180:
        error = 360 - error
    
    print(f"Error angular: {error:.2f}°")
    
    # Factor de corrección (si error es positivo, el robot gira de más)
    # Si medimos 360° pero realmente gira 370°, factor = 360/370
    measured_total = 360.0 + error
    correction_factor = 360.0 / measured_total
    
    print(f"Factor de corrección angular: {correction_factor:.4f}")
    
    # Actualizar config
    config = load_config()
    motion = config.setdefault('motion', {})
    motion['angular_scale'] = correction_factor
    save_config(config)
    
    print("✔ Calibración angular guardada en config.yaml")
    return correction_factor

async def calibrate_linear_1m(robot, distance_cm=100.0):
    """
    Calibra la escala lineal mediante avance de distancia conocida
    Retorna el factor de corrección
    """
    print(f"Calibración lineal ({distance_cm} cm)")
    print(f"El robot avanzará {distance_cm} cm y retrocederá...")
    print("Mide la distancia REAL recorrida y anótala.")
    
    # Resetear navegación
    await robot.reset_navigation()
    
    # Avanzar
    await robot.navigate_to(distance_cm, 0)
    await robot.wait(1.0)
    
    # Leer pose
    p_fwd = await robot.get_position()
    try:
        x_fwd, y_fwd = p_fwd.x, p_fwd.y
    except AttributeError:
        x_fwd, y_fwd = p_fwd[0], p_fwd[1]
    
    measured_dist = math.hypot(x_fwd, y_fwd)
    print(f"Distancia medida por odometría: {measured_dist:.2f} cm")
    
    # Solicitar medida real al usuario
    real_dist = input(f"Ingresa la distancia REAL medida con cinta métrica (cm): ").strip()
    try:
        real_dist = float(real_dist)
    except ValueError:
        print("❌ Valor inválido. No se actualizó la calibración.")
        return 1.0
    
    # Factor de corrección
    # Si comando 100cm pero real es 95cm, factor = 95/100 = 0.95
    correction_factor = real_dist / distance_cm
    
    print(f"Factor de corrección lineal: {correction_factor:.4f}")
    
    # Actualizar config
    config = load_config()
    motion = config.setdefault('motion', {})
    motion['linear_scale'] = correction_factor
    save_config(config)
    
    print("✔ Calibración lineal guardada en config.yaml")
    
    # Regresar al inicio
    await robot.navigate_to(0, 0)
    
    return correction_factor

if __name__ == "__main__":
    print("Módulo de calibración")
    print("Importa y usa calibrate_turn_360(robot) o calibrate_linear_1m(robot)")
