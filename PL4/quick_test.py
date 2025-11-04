"""
Script de prueba rápida del sistema de seguridad mejorado

Ejecuta una navegación corta para verificar que los umbrales
escalonados funcionan correctamente.

Uso:
    python quick_test.py
"""

import asyncio
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

import config
from potential_fields import combined_potential_speeds, reset_velocity_ramp
from sensor_logger import SensorLogger
from velocity_logger import VelocityLogger


robot = Create3(Bluetooth(config.BLUETOOTH_NAME))


@event(robot.when_play)
async def play(robot):
    """Test rápido de navegación con obstáculos"""
    
    print("\n" + "="*60)
    print("🧪 TEST RÁPIDO - Sistema de Seguridad Mejorado")
    print("="*60)
    print("Objetivo: Navegar 100 cm hacia adelante")
    print("Observar: Cambios de nivel de seguridad en logger")
    print("="*60 + "\n")
    
    # Resetear odometría
    await robot.reset_navigation()
    
    # Objetivo: 100 cm adelante
    q_goal = (0.0, 100.0)
    
    # Iniciar loggers
    sensor_logger = SensorLogger(robot, interval=0.5)
    sensor_logger.start()
    
    velocity_logger = VelocityLogger(potential_type='conic_test')
    velocity_logger.start()
    
    # Variables de control
    reset_velocity_ramp()
    iteration = 0
    max_iterations = 200  # ~10 segundos a 20 Hz
    
    print("▶️  Iniciando navegación...")
    print("   Coloca obstáculos en el camino para ver el sistema en acción\n")
    
    try:
        while iteration < max_iterations:
            # Leer posición
            pos = await robot.get_position()
            q = (pos.x, pos.y, pos.heading)
            
            # Leer sensores IR
            ir_prox = await robot.get_ir_proximity()
            ir_sensors = ir_prox.sensors if hasattr(ir_prox, 'sensors') else ir_prox
            
            # Calcular distancia a meta
            dx = q_goal[0] - q[0]
            dy = q_goal[1] - q[1]
            distance = (dx**2 + dy**2)**0.5
            
            # ¿Llegamos?
            if distance < config.TOL_DIST_CM:
                print("\n✅ META ALCANZADA")
                print(f"   Posición final: x={q[0]:.1f}, y={q[1]:.1f}")
                print(f"   Iteraciones: {iteration}")
                break
            
            # Calcular velocidades con sistema mejorado
            v_left, v_right, dist, info = combined_potential_speeds(
                q, q_goal, 
                ir_sensors=ir_sensors,
                potential_type='conic'
            )
            
            # Mostrar nivel de seguridad cada 10 iteraciones
            if iteration % 10 == 0:
                level = info.get('safety_level', 'N/A')
                v_max = info.get('v_max_allowed', 0)
                max_ir = info.get('max_ir_front', 0)
                print(f"[{iteration:3d}] Dist={distance:5.1f}cm  IR_max={max_ir:4d}  "
                      f"{level}  V_max={v_max:.1f}cm/s  V_actual={info['v_linear']:.1f}cm/s")
            
            # Aplicar velocidades
            await robot.set_wheel_speeds(v_left, v_right)
            
            # Log
            velocity_logger.log(
                position={'x': q[0], 'y': q[1], 'theta': q[2]},
                distance=distance,
                v_left=v_left,
                v_right=v_right,
                info=info
            )
            
            # Esperar
            await robot.wait(config.CONTROL_DT)
            iteration += 1
        
        # Parar robot
        await robot.set_wheel_speeds(0, 0)
        
        if iteration >= max_iterations:
            print("\n⏱️  Timeout alcanzado")
            print(f"   Distancia restante: {distance:.1f} cm")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        await robot.set_wheel_speeds(0, 0)
    
    finally:
        # Detener loggers
        sensor_logger.stop()
        velocity_logger.stop()
        
        print("\n" + "="*60)
        print("🏁 Test completado")
        print("="*60)
        print("📊 Revisa los archivos de log para análisis detallado")
        print("="*60 + "\n")


# Iniciar robot
robot.play()
