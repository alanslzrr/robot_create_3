# core/undock.py
# Rutina estandarizada de salida del Dock:
# 1) retrocede cm usando set_wheel_speeds (controlando ruedas directamente)
# 2) gira deg (por defecto 90, derecha)
# 3) reset_navigation()
# 4) feedback visual/sonoro y mensaje "listo para navegar"
#
# IMPORTANTE: Basado en ejemplos PL2 y manual_move.py
# El robot Create3 se controla con set_wheel_speeds(), NO con navigate_to()

import math

DEFAULT_BACK_CM = 30.0      # Retroceso desde dock (30 cm para salir completamente)
DEFAULT_TURN_DEG = 90.0
DEFAULT_TURN_DIR = "right"  # "right" or "left"
DEFAULT_BACK_SPEED = 5.0    # Velocidad de retroceso (cm/s)

async def perform_undock(robot,
                         back_cm: float = DEFAULT_BACK_CM,
                         turn_deg: float = DEFAULT_TURN_DEG,
                         turn_dir: str = DEFAULT_TURN_DIR,
                         back_speed: float = DEFAULT_BACK_SPEED):
    """
    Rutina de undock usando control directo de ruedas (set_wheel_speeds).
    
    Parámetros:
        robot: Instancia del robot Create3
        back_cm: Distancia a retroceder en cm (default: 30 cm)
        turn_deg: Ángulo de giro tras retroceder (default: 90°)
        turn_dir: Dirección de giro "right" o "left" (default: "right")
        back_speed: Velocidad de retroceso en cm/s (default: 5 cm/s)
    
    Nota: Esta implementación usa set_wheel_speeds() como en los ejemplos
    PL2/T02_Etapa01.py y examples/manual_move.py que funcionan correctamente.
    """
    # Señal de inicio
    await robot.set_lights_on_rgb(0, 128, 255)  # AZUL
    try:
        await robot.play_note(523, 0.08)  # C5
    except Exception:
        pass

    # Asegúrate que el (0,0) inicial es el dock actual
    await robot.reset_navigation()

    # 1) Retroceder back_cm usando control directo de ruedas
    # Ambas ruedas con velocidad negativa = retroceso recto
    # Tiempo = distancia / velocidad
    # Aplicar escala lineal si existe en config.motion
    try:
        # Intentar acceder a configuración de movimiento desde robot si fue inyectada
        from core.config_validator import get_validated_config
        _cfg = get_validated_config()
        lin_scale = float(_cfg.get('motion', {}).get('linear_scale', 1.0))
    except Exception:
        lin_scale = 1.0
    effective_speed = abs(back_speed) * lin_scale
    tiempo_retroceso = abs(back_cm) / max(effective_speed, 1e-6)
    
    print(f"→ Retrocediendo {back_cm:.1f} cm a {back_speed:.1f} cm/s durante {tiempo_retroceso:.2f}s")
    await robot.set_wheel_speeds(-effective_speed, -effective_speed)
    await robot.wait(tiempo_retroceso)
    await robot.set_wheel_speeds(0, 0)  # Detener
    print("✓ Retroceso completado")

    # 2) Girar usando los métodos nativos del SDK
    deg = abs(turn_deg)
    if (turn_dir or "").lower().startswith("l"):
        print(f"→ Girando {deg:.1f}° a la IZQUIERDA")
        await robot.turn_left(deg)
    else:
        print(f"→ Girando {deg:.1f}° a la DERECHA")
        await robot.turn_right(deg)
    print("✓ Giro completado")

    # 3) Reset para fijar esta pose como nuevo (0,0,θ)
    await robot.reset_navigation()
    print("✓ Navegación reseteada")

    # 4) Feedback final
    await robot.set_lights_on_rgb(0, 255, 0)  # VERDE
    try:
        await robot.play_note(659, 0.10)  # E5
    except Exception:
        pass
    print("✔ Listo para navegar (undock OK).")
