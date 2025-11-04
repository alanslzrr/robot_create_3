#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script de navegación con campos de potencial combinados (atractivo + repulsivo)

Autores: Yago Ramos - Salazar Alan
Fecha de finalización: 28 de octubre de 2025
Institución: UIE - Robots Autónomos
Robot SDK: irobot-edu-sdk

Objetivo:
    Implementar navegación autónoma combinando campo de potencial atractivo
    (que lleva al robot hacia la meta) con campo de potencial repulsivo
    (que evita obstáculos detectados por sensores IR). El sistema calcula
    la fuerza resultante y genera velocidades de rueda apropiadas para
    alcanzar el objetivo evitando colisiones.

Comportamiento esperado:
    - Cargar puntos de navegación desde points.json
    - Conectar con el robot vía Bluetooth y resetear odometría
    - Ejecutar bucle de control a 20 Hz (cada 50 ms):
        * Leer posición actual y sensores IR
        * Calcular fuerza atractiva hacia la meta
        * Calcular fuerza repulsiva de obstáculos detectados
        * Combinar ambas fuerzas y convertir a velocidades de rueda
        * Enviar comandos de velocidad al robot
        * Registrar datos en CSV para análisis
    - Detectar llegada a meta (distancia < 3 cm)
    - Manejar colisiones físicas con bumpers
    - Permitir interrupción segura con Ctrl+C

Diferencias con P01:
    - Usa combined_potential_speeds() en lugar de attractive_wheel_speeds()
    - Considera obstáculos detectados por sensores IR en tiempo real
    - Calcula fuerzas repulsivas inversamente proporcionales a distancia²
    - Combina vectores de fuerza atractiva y repulsiva
    - Registra información adicional sobre obstáculos en el CSV

Funciones de potencial soportadas:
    - linear: Proporcional a distancia (F = 0.25·d)
    - quadratic: Cuadrático (F = 0.01·d²/10)
    - conic: Con saturación (F = 0.15·min(d,100)·2)
    - exponential: Convergencia asintótica (F = 2.5·(1-e^(-d/50))·20)

Parámetros de potencial repulsivo:
    - K_REPULSIVE: Ganancia repulsiva (800.0 default)
    - D_INFLUENCE: Distancia de influencia (40.0 cm)
    - Basado en lecturas de sensores IR frontales (0-6)

Argumentos de línea de comandos:
    --potential: Tipo de función de potencial atractivo (default: linear)
    --robot: Nombre Bluetooth del robot (default: C3_UIEC_Grupo1)
    --points: Archivo JSON con waypoints (default: points.json)
    --k-rep: Ganancia repulsiva (default: 800.0)
    --d-influence: Distancia de influencia repulsiva (default: 40.0 cm)
    --debug: Mostrar información de debug cada 10 iteraciones

Módulos integrados:
    - potential_fields: Cálculo de fuerzas atractivas, repulsivas y combinadas
    - safety: Saturación y detección de emergencias
    - sensor_logger: Monitoreo de sensores en tiempo real
    - velocity_logger: Registro de datos de control en CSV

Salida:
    Archivo CSV en logs/ con timestamp único conteniendo:
    - Trayectoria completa (x, y, theta por iteración)
    - Velocidades calculadas (v_left, v_right, v_linear, omega)
    - Fuerzas repulsivas (fx, fy)
    - Número de obstáculos detectados
    - Tipo de función de potencial utilizada
"""

import argparse
import asyncio
import json
import math
import signal
import sys
from pathlib import Path

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import Create3

# Importar módulos propios
import config
from potential_fields import combined_potential_speeds, POTENTIAL_TYPES, reset_velocity_ramp
from safety import saturate_wheel_speeds, emergency_stop_needed, apply_obstacle_slowdown
from sensor_logger import SensorLogger
from velocity_logger import VelocityLogger


# ══════════════════════════════════════════════════════════
#  FUNCIONES AUXILIARES
# ══════════════════════════════════════════════════════════

def load_points(filename):
    """
    Carga q_i y q_f desde JSON.
    
    Returns:
        (q_i, q_f): Tuplas con (x, y, theta) y (x, y)
    """
    filepath = Path(filename)
    
    if not filepath.exists():
        print(f"\n❌ ERROR: {filename} no existe")
        print(f"   Ejecuta primero: python point_manager.py")
        sys.exit(1)
    
    try:
        data = json.loads(filepath.read_text())
    except json.JSONDecodeError as e:
        print(f"\n❌ ERROR: {filename} tiene formato JSON inválido")
        print(f"   {e}")
        sys.exit(1)
    
    # Validar estructura
    if 'q_i' not in data or 'q_f' not in data:
        print(f"\n❌ ERROR: {filename} debe contener 'q_i' y 'q_f'")
        sys.exit(1)
    
    # Extraer puntos
    q_i_data = data['q_i']
    q_f_data = data['q_f']
    
    q_i = (q_i_data['x'], q_i_data['y'], q_i_data['theta'])
    q_f = (q_f_data['x'], q_f_data['y'])
    
    # Validar distancia
    distance = math.hypot(q_f[0] - q_i[0], q_f[1] - q_i[1])
    if distance < 5.0:
        print(f"\n⚠️  ADVERTENCIA: q_i y q_f están muy cerca ({distance:.1f} cm)")
        print("   Considera definir puntos más separados")
    
    return q_i, q_f


def print_mission_info(q_i, q_f, robot_name, potential_type='linear', k_rep=None, d_influence=None):
    """Imprime información de la misión"""
    distance = math.hypot(q_f[0] - q_i[0], q_f[1] - q_i[1])
    angle = math.degrees(math.atan2(q_f[1] - q_i[1], q_f[0] - q_i[0]))
    
    # Seleccionar K según tipo de potencial
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
    
    if k_rep is None:
        k_rep = config.K_REPULSIVE
    if d_influence is None:
        d_influence = config.D_INFLUENCE
    
    print("\n" + "="*60)
    print("🚀 NAVEGACIÓN CON POTENCIAL COMBINADO - Parte 3.2")
    print("="*60)
    print(f"Robot: {robot_name}")
    print(f"Potencial Atractivo: {potential_type.upper()}")
    print(f"\n📍 Punto Inicial (q_i):")
    print(f"   x = {q_i[0]:.2f} cm")
    print(f"   y = {q_i[1]:.2f} cm")
    print(f"   θ = {q_i[2]:.1f}°")
    print(f"\n🎯 Punto Final (q_f):")
    print(f"   x = {q_f[0]:.2f} cm")
    print(f"   y = {q_f[1]:.2f} cm")
    print(f"\n📏 Distancia a recorrer: {distance:.1f} cm")
    print(f"📐 Ángulo hacia meta: {angle:.1f}°")
    print(f"\n⚙️  Parámetros de control:")
    print(f"   K_atractivo = {k_lin}")
    print(f"   K_angular = {config.K_ANGULAR}")
    print(f"   K_repulsivo = {k_rep}")
    print(f"   D_influencia = {d_influence} cm")
    print(f"   V_max = {config.V_MAX_CM_S} cm/s")
    print(f"   Tolerancia = {config.TOL_DIST_CM} cm")
    print("="*60 + "\n")


# ══════════════════════════════════════════════════════════
#  CONTROLADOR PRINCIPAL
# ══════════════════════════════════════════════════════════

class CombinedPotentialNavigator:
    """
    Navegador basado en campo de potencial combinado (atractivo + repulsivo).
    Integra sensores IR, seguridad y control.
    """
    
    def __init__(self, robot, q_goal, potential_type='linear', k_rep=None, d_influence=None, debug=False):
        self.robot = robot
        self.q_goal = q_goal
        self.potential_type = potential_type
        self.k_rep = k_rep or config.K_REPULSIVE
        self.d_influence = d_influence or config.D_INFLUENCE
        self.debug = debug
        self.logger = SensorLogger(robot)
        self.vel_logger = VelocityLogger(f"{potential_type}_combined")
        self.running = False
        
    async def navigate(self):
        """
        Ejecuta la navegación desde la posición actual hasta q_goal usando
        potencial combinado (atractivo + repulsivo).
        
        Returns:
            bool: True si llegó exitosamente, False si hubo error
        """
        # Resetear odometría
        await self.robot.reset_navigation()
        print(f"🔄 Navegación iniciada con potencial combinado: {self.potential_type}\n")
        
        # Resetear rampa de aceleración
        reset_velocity_ramp()
        
        # Iniciar loggers
        self.logger.start()
        self.vel_logger.start()
        self.running = True
        
        iteration = 0
        collision_count = 0
        MAX_COLLISIONS = 3
        
        try:
            while self.running:
                iteration += 1
                
                # Leer estado
                pos = await self.robot.get_position()
                q = (pos.x, pos.y, pos.heading)
                
                ir_prox = await self.robot.get_ir_proximity()
                ir_sensors = ir_prox.sensors if hasattr(ir_prox, 'sensors') else []
                bumpers = await self.robot.get_bumpers()
                
                # Calcular potencial COMBINADO (atractivo + repulsivo)
                v_left, v_right, distance, info = combined_potential_speeds(
                    q, self.q_goal, 
                    ir_sensors=ir_sensors,
                    k_rep=self.k_rep,
                    d_influence=self.d_influence,
                    potential_type=self.potential_type
                )
                
                # NO aplicar slowdown por IR - solo evadir con potencial repulsivo
                # El robot debe seguir avanzando y esquivando, no detenerse
                
                # LOG DE VELOCIDADES
                self.vel_logger.log(
                    {'x': pos.x, 'y': pos.y, 'theta': pos.heading},
                    distance, v_left, v_right, info
                )
                
                # Verificar llegada
                if distance < config.TOL_DIST_CM:
                    await self.robot.set_wheel_speeds(0, 0)
                    self.logger.stop()
                    self.vel_logger.stop()
                    
                    dist_traveled = math.hypot(pos.x, pos.y)
                    
                    print("\n" + "="*60)
                    print("🎯 META ALCANZADA")
                    print("="*60)
                    print(f"Potencial usado: {self.potential_type} + repulsivo")
                    print(f"Posición final: x={pos.x:.1f}, y={pos.y:.1f}, θ={pos.heading:.1f}°")
                    print(f"Distancia recorrida: {dist_traveled:.1f} cm")
                    print(f"Error a meta: {distance:.1f} cm")
                    print("="*60)
                    return True
                
                # EMERGENCIA: Colisión física
                if emergency_stop_needed(bumpers):
                    collision_count += 1
                    await self.robot.set_wheel_speeds(0, 0)
                    print(f"\n🚨 COLISIÓN {collision_count}/{MAX_COLLISIONS}")
                    
                    if collision_count >= MAX_COLLISIONS:
                        print(f"❌ Camino bloqueado - demasiadas colisiones")
                        self.logger.stop()
                        self.vel_logger.stop()
                        return False
                    
                    # Retroceder un poco
                    print("   Retrocediendo...")
                    await self.robot.set_wheel_speeds(-10, -10)
                    await self.robot.wait(1.0)
                    await self.robot.set_wheel_speeds(0, 0)
                    await self.robot.wait(0.5)
                    continue
                
                # Saturar velocidades (ya combinadas)
                v_left, v_right = saturate_wheel_speeds(v_left, v_right)
                
                # Debug
                if self.debug and iteration % 10 == 0:
                    num_obs = info.get('num_obstacles', 0)
                    fx_rep = info.get('fx_repulsive', 0.0)
                    fy_rep = info.get('fy_repulsive', 0.0)
                    print(f"[{iteration:04d}] d={distance:5.1f} obs={num_obs} "
                          f"F_rep=({fx_rep:6.1f},{fy_rep:6.1f}) "
                          f"v_l={v_left:5.1f} v_r={v_right:5.1f}")
                
                await self.robot.set_wheel_speeds(v_left, v_right)
                await self.robot.wait(config.CONTROL_DT)
        
        except Exception as e:
            print(f"\n❌ Error durante navegación: {e}")
            import traceback
            traceback.print_exc()
            await self.robot.set_wheel_speeds(0, 0)
            self.logger.stop()
            self.vel_logger.stop()
            return False
        
        return False


# ══════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════

def main():
    """Función principal"""
    
    # Parsear argumentos
    parser = argparse.ArgumentParser(
        description="Navegación con potencial combinado (atractivo + repulsivo) para iRobot Create 3",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos de uso:
  python PRM01_P02.py
  python PRM01_P02.py --debug
  python PRM01_P02.py --potential quadratic --k-rep 1000
  python PRM01_P02.py --robot "MiRobot" --d-influence 50
        """
    )
    parser.add_argument(
        "--robot",
        default=config.BLUETOOTH_NAME,
        help=f"Nombre Bluetooth del robot (default: {config.BLUETOOTH_NAME})"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Mostrar información de debug cada 10 iteraciones"
    )
    parser.add_argument(
        "--points",
        default=config.POINTS_FILE,
        help=f"Archivo JSON con puntos (default: {config.POINTS_FILE})"
    )
    parser.add_argument(
        "--potential",
        choices=POTENTIAL_TYPES,
        default='linear',
        help=f"Tipo de función de potencial atractivo (default: linear)"
    )
    parser.add_argument(
        "--k-rep",
        type=float,
        default=config.K_REPULSIVE,
        help=f"Ganancia repulsiva (default: {config.K_REPULSIVE})"
    )
    parser.add_argument(
        "--d-influence",
        type=float,
        default=config.D_INFLUENCE,
        help=f"Distancia de influencia repulsiva en cm (default: {config.D_INFLUENCE})"
    )
    
    args = parser.parse_args()
    
    # Cargar puntos
    q_i, q_f = load_points(args.points)
    
    # Mostrar información
    print_mission_info(q_i, q_f, args.robot, args.potential, 
                      k_rep=args.k_rep, d_influence=args.d_influence)
    
    # Conectar al robot
    print(f"🔌 Conectando a '{args.robot}'...")
    try:
        robot = Create3(Bluetooth(args.robot))
    except Exception as e:
        print(f"\n❌ Error de conexión Bluetooth:")
        print(f"   {e}")
        print(f"\nVerifica que:")
        print(f"  1. El robot esté encendido")
        print(f"  2. Bluetooth esté habilitado en tu PC")
        print(f"  3. El nombre '{args.robot}' sea correcto")
        sys.exit(1)
    
    # Variable para control de emergencia
    navigator = None
    
    # Handler para Ctrl+C
    def emergency_shutdown(signum, frame):
        print("\n\n🛑 INTERRUPCIÓN MANUAL - Deteniendo robot...")
        
        async def _stop():
            try:
                await robot.set_wheel_speeds(0, 0)
                if navigator:
                    navigator.logger.stop()
            except:
                pass
            finally:
                print("✅ Robot detenido")
                sys.exit(0)
        
        asyncio.run(_stop())
    
    signal.signal(signal.SIGINT, emergency_shutdown)
    
    # Variable de resultado
    mission_success = False
    
    @robot.when_play
    async def start_navigation(robot):
        nonlocal navigator, mission_success
        navigator = CombinedPotentialNavigator(
            robot, q_f, 
            potential_type=args.potential,
            k_rep=args.k_rep,
            d_influence=args.d_influence,
            debug=args.debug
        )
        mission_success = await navigator.navigate()
        
        if mission_success:
            print("\n✅ Misión completada")
        else:
            print("\n❌ Misión fallida")
    
    print("▶️  Iniciando...\n")
    robot.play()  # Bloqueante


if __name__ == "__main__":
    main()
