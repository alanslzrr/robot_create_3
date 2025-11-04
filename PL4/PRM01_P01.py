#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script principal de navegación con campos de potencial atractivo - Parte 01

Autores: Alan Salazar, Yago Ramos
Fecha: 4 de noviembre de 2025
Institución: UIE Universidad Intercontinental de la Empresa
Asignatura: Robots Autónomos - Profesor Eladio Dapena
Robot SDK: irobot-edu-sdk

OBJETIVOS PRINCIPALES:

En este script implementamos la primera parte de la práctica evaluada número 4,
enfocada en la navegación autónoma utilizando únicamente campos de potencial
atractivo. Nuestro objetivo principal era desarrollar un sistema que permitiera
al robot navegar desde una posición inicial hasta una posición final usando
diferentes funciones de potencial, cada una con características específicas
de comportamiento.

Los objetivos específicos que buscamos alcanzar incluyen:

1. Implementar un sistema de navegación que calcule velocidades de rueda basadas
   en campos de potencial atractivo hacia el objetivo
2. Proporcionar cuatro funciones de potencial diferentes (lineal, cuadrática,
   cónica y exponencial) para permitir análisis comparativo
3. Integrar sistemas de seguridad que protejan al robot mediante detección de
   obstáculos con sensores IR y manejo de colisiones físicas
4. Registrar todos los datos de navegación en archivos CSV para análisis
   posterior y comparación entre diferentes funciones de potencial
5. Implementar un sistema robusto de control que funcione a 20 Hz con manejo
   adecuado de errores y interrupciones

CONFIGURACIÓN:

El script está configurado para trabajar con el módulo config.py que contiene
todos los parámetros del sistema. Los principales parámetros configurables
incluyen:

- Velocidades máximas y mínimas del robot
- Ganancias de control específicas para cada tipo de potencial
- Umbrales de detección de obstáculos mediante sensores IR
- Período de control (50 ms para 20 Hz)
- Tolerancia de llegada a la meta (3 cm)

El script carga los puntos de navegación desde un archivo JSON ubicado en
data/points.json, que debe contener las coordenadas del punto inicial (q_i)
con posición y orientación, y el punto final (q_f) con solo posición.

El sistema utiliza argumentos de línea de comandos para permitir flexibilidad
en la ejecución, permitiendo seleccionar el tipo de potencial, el nombre del
robot Bluetooth, y activar modo debug para monitoreo detallado.
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

# Importar módulos propios del sistema
from src import config
from src.potential_fields import attractive_wheel_speeds, POTENTIAL_TYPES, reset_velocity_ramp
from src.safety import saturate_wheel_speeds, detect_obstacle, emergency_stop_needed, apply_obstacle_slowdown
from src.sensor_logger import SensorLogger
from src.velocity_logger import VelocityLogger


# ══════════════════════════════════════════════════════════
#  FUNCIONES AUXILIARES
# ══════════════════════════════════════════════════════════

def load_points(filename):
    """
    Carga los puntos de navegación desde un archivo JSON.
    
    Esta función lee el archivo JSON que contiene las coordenadas del punto
    inicial (q_i) y el punto final (q_f). Validamos que el archivo exista,
    que tenga formato JSON válido, y que contenga la estructura esperada con
    las claves q_i y q_f. También verificamos que los puntos no estén demasiado
    cerca entre sí, ya que esto podría causar problemas en la navegación.
    
    Args:
        filename: Ruta al archivo JSON con los puntos de navegación
    
    Returns:
        tuple: Tupla con (q_i, q_f) donde:
            - q_i: Tupla (x, y, theta) con posición y orientación inicial
            - q_f: Tupla (x, y) con posición final
    
    Raises:
        SystemExit: Si el archivo no existe, tiene formato inválido, o no
                    contiene la estructura esperada
    """
    filepath = Path(filename)
    
    # Verificar que el archivo exista
    if not filepath.exists():
        print(f"\n[ERROR] {filename} no existe")
        print(f"        Ejecuta primero: python utils/point_manager.py")
        sys.exit(1)
    
    # Intentar cargar y parsear el JSON
    try:
        data = json.loads(filepath.read_text())
    except json.JSONDecodeError as e:
        print(f"\n[ERROR] {filename} tiene formato JSON inválido")
        print(f"        {e}")
        sys.exit(1)
    
    # Validar que contenga las claves necesarias
    if 'q_i' not in data or 'q_f' not in data:
        print(f"\n[ERROR] {filename} debe contener 'q_i' y 'q_f'")
        sys.exit(1)
    
    # Extraer los puntos de la estructura JSON
    q_i_data = data['q_i']
    q_f_data = data['q_f']
    
    q_i = (q_i_data['x'], q_i_data['y'], q_i_data['theta'])
    q_f = (q_f_data['x'], q_f_data['y'])
    
    # Validar que los puntos no estén demasiado cerca
    distance = math.hypot(q_f[0] - q_i[0], q_f[1] - q_i[1])
    if distance < 5.0:
        print(f"\n[WARNING] q_i y q_f están muy cerca ({distance:.1f} cm)")
        print("          Considera definir puntos más separados")
    
    return q_i, q_f


def print_mission_info(q_i, q_f, robot_name, potential_type='linear'):
    """
    Imprime información detallada sobre la misión de navegación.
    
    Esta función muestra un resumen completo de los parámetros de la misión
    antes de iniciar la navegación. Calculamos la distancia a recorrer y el
    ángulo hacia la meta, y seleccionamos la ganancia lineal apropiada según
    el tipo de potencial elegido. La información mostrada incluye los puntos
    inicial y final, los parámetros de control configurados, y los límites
    de seguridad del sistema.
    
    Args:
        q_i: Tupla (x, y, theta) con posición y orientación inicial
        q_f: Tupla (x, y) con posición final
        robot_name: Nombre Bluetooth del robot a conectar
        potential_type: Tipo de función de potencial a utilizar
    """
    # Calcular distancia y ángulo hacia la meta
    distance = math.hypot(q_f[0] - q_i[0], q_f[1] - q_i[1])
    angle = math.degrees(math.atan2(q_f[1] - q_i[1], q_f[0] - q_i[0]))
    
    # Seleccionar la ganancia lineal apropiada según el tipo de potencial
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
    
    # Mostrar información formateada
    print("\n" + "="*60)
    print("NAVEGACION CON POTENCIAL ATRACTIVO - Parte 3.1")
    print("="*60)
    print(f"Robot: {robot_name}")
    print(f"Potencial: {potential_type.upper()}")
    print(f"\nPunto Inicial (q_i):")
    print(f"   x = {q_i[0]:.2f} cm")
    print(f"   y = {q_i[1]:.2f} cm")
    print(f"   theta = {q_i[2]:.1f} deg")
    print(f"\nPunto Final (q_f):")
    print(f"   x = {q_f[0]:.2f} cm")
    print(f"   y = {q_f[1]:.2f} cm")
    print(f"\nDistancia a recorrer: {distance:.1f} cm")
    print(f"Angulo hacia meta: {angle:.1f} deg")
    print(f"\nParametros de control:")
    print(f"   K_lineal = {k_lin}")
    print(f"   K_angular = {config.K_ANGULAR}")
    print(f"   V_max = {config.V_MAX_CM_S} cm/s")
    print(f"   Tolerancia = {config.TOL_DIST_CM} cm")
    print("="*60 + "\n")


# ══════════════════════════════════════════════════════════
#  CONTROLADOR PRINCIPAL
# ══════════════════════════════════════════════════════════

class AttractiveFieldNavigator:
    """
    Navegador basado en campo de potencial atractivo.
    
    Esta clase implementa el controlador principal para la navegación usando
    únicamente campos de potencial atractivo. Integra todos los componentes
    del sistema incluyendo sensores, seguridad, logging y control de velocidad.
    El navegador ejecuta un bucle de control continuo que calcula velocidades
    basadas en la función de potencial seleccionada y aplica correcciones de
    seguridad según las lecturas de los sensores.
    
    El sistema funciona a 20 Hz (cada 50 ms) y en cada iteración lee la
    posición actual del robot, calcula las velocidades necesarias para avanzar
    hacia el objetivo, aplica reducciones por detección de obstáculos mediante
    sensores IR, satura las velocidades dentro de límites seguros, y envía los
    comandos a las ruedas del robot.
    """
    
    def __init__(self, robot, q_goal, potential_type='linear', debug=False):
        """
        Inicializa el navegador con los parámetros de configuración.
        
        Creamos las instancias de los loggers de sensores y velocidades, y
        almacenamos las referencias al robot y al objetivo. También configuramos
        el tipo de potencial a utilizar y el modo debug.
        
        Args:
            robot: Instancia del robot Create3 conectado
            q_goal: Tupla (x, y) con coordenadas del objetivo
            potential_type: Tipo de función de potencial ('linear', 'quadratic',
                           'conic', 'exponential')
            debug: Si es True, muestra información detallada cada 10 iteraciones
        """
        self.robot = robot
        self.q_goal = q_goal
        self.potential_type = potential_type
        self.debug = debug
        self.logger = SensorLogger(robot)
        self.vel_logger = VelocityLogger(potential_type)
        self.running = False
        
    async def navigate(self):
        """
        Ejecuta el bucle principal de navegación hasta alcanzar el objetivo.
        
        Este método implementa el bucle de control completo de la navegación.
        En cada iteración realizamos las siguientes operaciones:
        
        1. Leemos la posición actual del robot mediante odometría
        2. Leemos los sensores IR y los bumpers para detección de obstáculos
        3. Calculamos las velocidades de rueda usando la función de potencial
           atractivo seleccionada
        4. Registramos los datos en el archivo CSV para análisis posterior
        5. Verificamos si hemos alcanzado la meta (distancia < 3 cm)
        6. Verificamos colisiones físicas mediante bumpers
        7. Aplicamos reducción de velocidad por detección de obstáculos IR
        8. Saturaremos las velocidades dentro de límites seguros
        9. Enviamos los comandos de velocidad al robot
        
        El sistema maneja colisiones físicas permitiendo hasta 3 intentos antes
        de abortar la misión. También registra todos los datos en archivos CSV
        para permitir análisis comparativo posterior.
        
        Returns:
            bool: True si llegó exitosamente al objetivo, False si hubo error
                  o se abortó la misión
        """
        # Resetear la odometría del robot al inicio de la navegación
        await self.robot.reset_navigation()
        print(f"[INFO] Navegacion iniciada con potencial: {self.potential_type}\n")
        
        # Resetear la rampa de aceleración para empezar desde velocidad cero
        reset_velocity_ramp()
        
        # Iniciar los sistemas de logging en segundo plano
        self.logger.start()
        self.vel_logger.start()
        self.running = True
        
        # Variables para control de iteraciones y colisiones
        iteration = 0
        collision_count = 0
        MAX_COLLISIONS = 3
        
        try:
            while self.running:
                iteration += 1
                
                # Leer el estado actual del robot
                pos = await self.robot.get_position()
                q = (pos.x, pos.y, pos.heading)
                
                # Leer sensores para detección de obstáculos
                ir_prox = await self.robot.get_ir_proximity()
                ir_sensors = ir_prox.sensors if hasattr(ir_prox, 'sensors') else []
                bumpers = await self.robot.get_bumpers()
                
                # Calcular velocidades usando la función de potencial atractivo
                # seleccionada. Esta función retorna las velocidades de rueda,
                # la distancia al objetivo, y información adicional para logging
                v_left, v_right, distance, info = attractive_wheel_speeds(
                    q, self.q_goal, potential_type=self.potential_type
                )
                
                # Registrar los datos de esta iteración en el archivo CSV
                self.vel_logger.log(
                    {'x': pos.x, 'y': pos.y, 'theta': pos.heading},
                    distance, v_left, v_right, info
                )
                
                # Verificar si hemos alcanzado la meta
                if distance < config.TOL_DIST_CM:
                    await self.robot.set_wheel_speeds(0, 0)
                    self.logger.stop()
                    self.vel_logger.stop()
                    
                    # Calcular la distancia total recorrida desde el origen
                    dist_traveled = math.hypot(pos.x, pos.y)
                    
                    # Mostrar resumen de la misión completada
                    print("\n" + "="*60)
                    print("[SUCCESS] META ALCANZADA")
                    print("="*60)
                    print(f"Potencial usado: {self.potential_type}")
                    print(f"Posicion final: x={pos.x:.1f}, y={pos.y:.1f}, theta={pos.heading:.1f} deg")
                    print(f"Distancia recorrida: {dist_traveled:.1f} cm")
                    print(f"Error a meta: {distance:.1f} cm")
                    print("="*60)
                    return True
                
                # Manejo de emergencias: colisión física detectada por bumpers
                if emergency_stop_needed(bumpers):
                    collision_count += 1
                    await self.robot.set_wheel_speeds(0, 0)
                    print(f"\n[COLLISION] Colision {collision_count}/{MAX_COLLISIONS} detectada")
                    
                    # Si excedemos el número máximo de colisiones, abortamos
                    if collision_count >= MAX_COLLISIONS:
                        print(f"[ERROR] Camino bloqueado")
                        self.logger.stop()
                        self.vel_logger.stop()
                        return False
                    
                    # Esperar un momento antes de continuar después de la colisión
                    await self.robot.wait(1.5)
                    continue
                
                # Aplicar reducción de velocidad si detectamos obstáculos mediante IR
                # Esto proporciona una capa adicional de seguridad antes de que
                # ocurra una colisión física
                v_left, v_right, obs = apply_obstacle_slowdown(v_left, v_right, ir_sensors)
                
                # Saturar las velocidades dentro de los límites seguros del robot
                v_left, v_right = saturate_wheel_speeds(v_left, v_right)
                
                # Mostrar información de debug si está habilitado
                if self.debug and iteration % 10 == 0:
                    print(f"[{iteration:04d}] d={distance:5.1f} v_l={v_left:5.1f} v_r={v_right:5.1f}")
                
                # Enviar comandos de velocidad a las ruedas del robot
                await self.robot.set_wheel_speeds(v_left, v_right)
                
                # Esperar el período de control antes de la siguiente iteración
                await self.robot.wait(config.CONTROL_DT)
        
        except Exception as e:
            # Manejo de errores durante la navegación
            print(f"\n[ERROR] Error durante navegacion: {e}")
            await self.robot.set_wheel_speeds(0, 0)
            self.logger.stop()
            self.vel_logger.stop()
            return False
        
        return False


# ══════════════════════════════════════════════════════════
#  FUNCIÓN PRINCIPAL
# ══════════════════════════════════════════════════════════

def main():
    """
    Función principal que inicializa y ejecuta el sistema de navegación.
    
    Esta función se encarga de configurar todos los componentes necesarios
    para la navegación. Primero parseamos los argumentos de línea de comandos
    para permitir configurar el tipo de potencial, el nombre del robot, y
    otras opciones. Luego cargamos los puntos de navegación desde el archivo
    JSON, establecemos la conexión Bluetooth con el robot, y configuramos el
    manejo de interrupciones para permitir detención segura con Ctrl+C.
    
    El sistema utiliza el evento when_play del SDK del robot para iniciar
    la navegación una vez que se establece la conexión. Todo el control se
    ejecuta de forma asíncrona utilizando asyncio.
    """
    
    # Configurar el parser de argumentos de línea de comandos
    parser = argparse.ArgumentParser(
        description="Navegación con potencial atractivo para iRobot Create 3",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos de uso:
  python PRM01_P01.py
  python PRM01_P01.py --debug
  python PRM01_P01.py --robot "MiRobot"
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
        default=f"data/{config.POINTS_FILE}",
        help=f"Archivo JSON con puntos (default: data/{config.POINTS_FILE})"
    )
    parser.add_argument(
        "--potential",
        choices=POTENTIAL_TYPES,
        default='linear',
        help=f"Tipo de función de potencial (default: linear)"
    )
    
    # Parsear los argumentos proporcionados
    args = parser.parse_args()
    
    # Cargar los puntos de navegación desde el archivo JSON
    q_i, q_f = load_points(args.points)
    
    # Mostrar información de la misión antes de iniciar
    print_mission_info(q_i, q_f, args.robot, args.potential)
    
    # Establecer conexión Bluetooth con el robot
    print(f"[CONNECTING] Conectando a '{args.robot}'...")
    try:
        robot = Create3(Bluetooth(args.robot))
    except Exception as e:
        print(f"\n[ERROR] Error de conexion Bluetooth:")
        print(f"        {e}")
        print(f"\n        Verifica que:")
        print(f"        1. El robot este encendido")
        print(f"        2. Bluetooth este habilitado en tu PC")
        print(f"        3. El nombre '{args.robot}' sea correcto")
        sys.exit(1)
    
    # Variable para almacenar la referencia al navegador (necesaria para
    # el manejo de interrupciones)
    navigator = None
    
    # Configurar el manejador de señal para permitir interrupción segura
    # con Ctrl+C. Esto asegura que el robot se detenga correctamente si
    # el usuario interrumpe la ejecución
    def emergency_shutdown(signum, frame):
        print("\n\n[INTERRUPT] Interrupcion manual - Deteniendo robot...")
        
        async def _stop():
            try:
                await robot.set_wheel_speeds(0, 0)
                if navigator:
                    navigator.logger.stop()
            except:
                pass
            finally:
                print("[STOPPED] Robot detenido")
                sys.exit(0)
        
        asyncio.run(_stop())
    
    signal.signal(signal.SIGINT, emergency_shutdown)
    
    # Variable para almacenar el resultado de la misión
    mission_success = False
    
    # Configurar el callback que se ejecuta cuando el robot está listo
    # Este es el punto de entrada principal del SDK de iRobot
    @robot.when_play
    async def start_navigation(robot):
        nonlocal navigator, mission_success
        
        # Crear la instancia del navegador con los parámetros configurados
        navigator = AttractiveFieldNavigator(
            robot, q_f, 
            potential_type=args.potential,
            debug=args.debug
        )
        
        # Ejecutar la navegación y almacenar el resultado
        mission_success = await navigator.navigate()
        
        # Mostrar el resultado final de la misión
        if mission_success:
            print("\n[SUCCESS] Mision completada")
        else:
            print("\n[FAILED] Mision fallida")
    
    # Iniciar el robot. Esta llamada es bloqueante y mantiene el programa
    # ejecutándose hasta que termine la navegación o se interrumpa
    print("[STARTING] Iniciando...\n")
    robot.play()


if __name__ == "__main__":
    main()
