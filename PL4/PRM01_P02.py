#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script principal de navegación con campos de potencial combinados - Parte 02

Autores: Alan Salazar, Yago Ramos
Fecha: 4 de noviembre de 2025
Institución: UIE Universidad Intercontinental de la Empresa
Asignatura: Robots Autónomos - Profesor Eladio Dapena
Robot SDK: irobot-edu-sdk

OBJETIVOS PRINCIPALES:

En este script implementamos la segunda parte de la práctica evaluada número 4,
extendiendo la funcionalidad del script anterior para incluir campos de potencial
repulsivo que permiten al robot evitar obstáculos mientras navega hacia el objetivo.
Nuestro objetivo principal era desarrollar un sistema de navegación más robusto
que combinara la atracción hacia la meta con la repulsión de obstáculos detectados.

Los objetivos específicos que buscamos alcanzar incluyen:

1. Implementar un sistema de navegación que combine campos de potencial atractivo
   y repulsivo para lograr navegación autónoma con evasión de obstáculos
2. Utilizar los sensores infrarrojos del robot para detectar obstáculos en tiempo
   real y calcular fuerzas repulsivas apropiadas
3. Convertir las lecturas de sensores IR en posiciones estimadas de obstáculos
   utilizando un modelo físico basado en la relación inversa al cuadrado
4. Combinar vectorialmente las fuerzas atractivas y repulsivas para generar una
   dirección de movimiento resultante que evite colisiones
5. Implementar un sistema de control de velocidad dinámico que reduzca la velocidad
   según la proximidad de obstáculos detectados
6. Mantener la capacidad de análisis comparativo registrando datos adicionales
   sobre fuerzas repulsivas y obstáculos detectados

CONFIGURACIÓN:

El script está configurado de manera similar al anterior, pero con parámetros
adicionales para el potencial repulsivo. Los principales parámetros configurables
incluyen:

- Ganancias de potencial atractivo (iguales a la Parte 01)
- Ganancia repulsiva (K_REPULSIVE) que controla la intensidad de la evasión
- Distancia de influencia (D_INFLUENCE) que define el rango efectivo de los
  campos repulsivos
- Sistema de umbrales escalonados para control dinámico de velocidad según
  proximidad de obstáculos

El script utiliza los mismos archivos de configuración y puntos de navegación
que la Parte 01, pero ahora también acepta argumentos para ajustar los parámetros
del potencial repulsivo, permitiendo experimentar con diferentes configuraciones
según las características del entorno.

DIFERENCIAS CON LA PARTE 01:

La principal diferencia con respecto a PRM01_P01.py es que utilizamos la función
combined_potential_speeds() en lugar de attractive_wheel_speeds(). Esta función
no solo calcula la fuerza atractiva hacia el objetivo, sino que también:

- Lee las lecturas de los siete sensores infrarrojos del robot
- Estima las posiciones de obstáculos basándose en un modelo físico
- Calcula fuerzas repulsivas para cada obstáculo detectado
- Combina vectorialmente todas las fuerzas para obtener la dirección resultante
- Aplica un sistema de límites dinámicos de velocidad según la proximidad de
  obstáculos detectados

El sistema está diseñado para mantener al robot avanzando hacia el objetivo
mientras evita obstáculos, en lugar de detenerse completamente cuando detecta
un obstáculo cercano.
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
from src.potential_fields import combined_potential_speeds, POTENTIAL_TYPES, reset_velocity_ramp
from src.safety import saturate_wheel_speeds, emergency_stop_needed, apply_obstacle_slowdown
from src.sensor_logger import SensorLogger
from src.velocity_logger import VelocityLogger


# ══════════════════════════════════════════════════════════
#  FUNCIONES AUXILIARES
# ══════════════════════════════════════════════════════════

def load_points(filename):
    """
    Carga los puntos de navegación desde un archivo JSON.
    
    Esta función es idéntica a la de PRM01_P01.py y realiza las mismas
    validaciones. Leemos el archivo JSON que contiene las coordenadas del
    punto inicial (q_i) y el punto final (q_f), validamos su existencia y
    formato, y extraemos los valores necesarios para la navegación.
    
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


def print_mission_info(q_i, q_f, robot_name, potential_type='linear', k_rep=None, d_influence=None):
    """
    Imprime información detallada sobre la misión de navegación.
    
    Similar a la función en PRM01_P01.py, pero ahora también muestra los
    parámetros del potencial repulsivo. Calculamos la distancia a recorrer
    y el ángulo hacia la meta, seleccionamos la ganancia lineal apropiada
    según el tipo de potencial elegido, y mostramos tanto los parámetros
    atractivos como los repulsivos.
    
    Args:
        q_i: Tupla (x, y, theta) con posición y orientación inicial
        q_f: Tupla (x, y) con posición final
        robot_name: Nombre Bluetooth del robot a conectar
        potential_type: Tipo de función de potencial atractivo a utilizar
        k_rep: Ganancia repulsiva (usa el valor por defecto si es None)
        d_influence: Distancia de influencia repulsiva (usa el valor por
                     defecto si es None)
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
    
    # Usar valores por defecto si no se especificaron
    if k_rep is None:
        k_rep = config.K_REPULSIVE
    if d_influence is None:
        d_influence = config.D_INFLUENCE
    
    # Mostrar información formateada incluyendo parámetros repulsivos
    print("\n" + "="*60)
    print("NAVEGACION CON POTENCIAL COMBINADO - Parte 3.2")
    print("="*60)
    print(f"Robot: {robot_name}")
    print(f"Potencial Atractivo: {potential_type.upper()}")
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
    
    Esta clase implementa el controlador principal para la navegación usando
    campos de potencial combinados. A diferencia de AttractiveFieldNavigator,
    esta clase integra tanto las fuerzas atractivas hacia el objetivo como las
    fuerzas repulsivas de los obstáculos detectados mediante sensores IR.
    
    El sistema funciona leyendo continuamente los siete sensores infrarrojos
    del robot, estimando las posiciones de obstáculos basándose en un modelo
    físico que relaciona la intensidad de la señal con la distancia, y luego
    calculando fuerzas repulsivas que se combinan vectorialmente con la fuerza
    atractiva hacia el objetivo. El resultado es una navegación que se ajusta
    dinámicamente para evitar colisiones mientras mantiene el objetivo de llegar
    a la meta.
    """
    
    def __init__(self, robot, q_goal, potential_type='linear', k_rep=None, d_influence=None, debug=False):
        """
        Inicializa el navegador con los parámetros de configuración.
        
        Creamos las instancias de los loggers y almacenamos las referencias
        necesarias. Además de los parámetros básicos, también almacenamos los
        parámetros específicos del potencial repulsivo que pueden ser ajustados
        mediante argumentos de línea de comandos.
        
        Args:
            robot: Instancia del robot Create3 conectado
            q_goal: Tupla (x, y) con coordenadas del objetivo
            potential_type: Tipo de función de potencial atractivo
            k_rep: Ganancia repulsiva (usa config.K_REPULSIVE si es None)
            d_influence: Distancia de influencia repulsiva (usa config.D_INFLUENCE
                        si es None)
            debug: Si es True, muestra información detallada cada 10 iteraciones
        """
        self.robot = robot
        self.q_goal = q_goal
        self.potential_type = potential_type
        self.k_rep = k_rep or config.K_REPULSIVE
        self.d_influence = d_influence or config.D_INFLUENCE
        self.debug = debug
        self.logger = SensorLogger(robot)
        self.vel_logger = VelocityLogger(f"{potential_type}_combined")
        self.running = False
        self.current_led_color = None  # Para rastrear el color actual del LED
        self.obstacle_detected = False  # Para rastrear si ya se detectó obstáculo (para sonido)
        
    async def navigate(self):
        """
        Ejecuta el bucle principal de navegación usando potencial combinado.
        
        Este método implementa el bucle de control completo para navegación
        con evasión de obstáculos. En cada iteración realizamos las siguientes
        operaciones:
        
        1. Leemos la posición actual del robot mediante odometría
        2. Leemos los sensores IR para detectar obstáculos en tiempo real
        3. Leemos los bumpers para detectar colisiones físicas
        4. Calculamos las velocidades usando la función combined_potential_speeds()
           que combina fuerzas atractivas y repulsivas
        5. Registramos los datos incluyendo información sobre obstáculos detectados
        6. Verificamos si hemos alcanzado la meta
        7. Manejamos colisiones físicas con retroceso automático
        8. Saturaremos las velocidades dentro de límites seguros
        9. Enviamos los comandos de velocidad al robot
        
        La diferencia clave con respecto a la Parte 01 es que no aplicamos
        reducción de velocidad adicional por detección IR, ya que el potencial
        repulsivo ya maneja la evasión. El robot debe seguir avanzando y
        esquivando, no detenerse cuando detecta obstáculos.
        
        Returns:
            bool: True si llegó exitosamente al objetivo, False si hubo error
                  o se abortó la misión
        """
        # Resetear la odometría del robot al inicio de la navegación
        await self.robot.reset_navigation()
        print(f"[INFO] Navegacion iniciada con potencial combinado: {self.potential_type}\n")
        
        # Resetear la rampa de aceleración para empezar desde velocidad cero
        reset_velocity_ramp()
        
        # LED VERDE: Listo para iniciar (estado inicial)
        await self.robot.set_lights_rgb(0, 255, 0)
        self.current_led_color = 'green'
        await self.robot.wait(1.0)  # Mantener verde 1 segundo antes de empezar
        
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
                
                # Leer sensores IR para detección de obstáculos en tiempo real
                ir_prox = await self.robot.get_ir_proximity()
                ir_sensors = ir_prox.sensors if hasattr(ir_prox, 'sensors') else []
                bumpers = await self.robot.get_bumpers()
                
                # Calcular velocidades usando potencial COMBINADO (atractivo + repulsivo)
                # Esta función toma en cuenta las lecturas IR para calcular obstáculos
                # y generar fuerzas repulsivas que modifican la trayectoria
                v_left, v_right, distance, info = combined_potential_speeds(
                    q, self.q_goal, 
                    ir_sensors=ir_sensors,
                    k_rep=self.k_rep,
                    d_influence=self.d_influence,
                    potential_type=self.potential_type
                )
                
                # IMPORTANTE: No aplicamos slowdown adicional por IR aquí porque
                # el potencial repulsivo ya maneja la evasión. El robot debe seguir
                # avanzando y esquivando, no detenerse cuando detecta obstáculos.
                # Esto permite una navegación más fluida y efectiva.
                
                # Registrar los datos de esta iteración en el archivo CSV
                # Incluimos información adicional sobre fuerzas repulsivas y obstáculos
                self.vel_logger.log(
                    {'x': pos.x, 'y': pos.y, 'theta': pos.heading},
                    distance, v_left, v_right, info
                )
                
                # Verificar si hemos alcanzado la meta
                if distance < config.TOL_DIST_CM:
                    await self.robot.set_wheel_speeds(0, 0)
                    
                    # LED VERDE: Meta alcanzada
                    await self.robot.set_lights_rgb(0, 255, 0)
                    
                    self.logger.stop()
                    self.vel_logger.stop()
                    
                    # Calcular la distancia total recorrida desde el origen
                    dist_traveled = math.hypot(pos.x, pos.y)
                    
                    # Mostrar resumen de la misión completada
                    print("\n" + "="*60)
                    print("[SUCCESS] META ALCANZADA")
                    print("="*60)
                    print(f"Potencial usado: {self.potential_type} + repulsivo")
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
                        print(f"[ERROR] Camino bloqueado - demasiadas colisiones")
                        self.logger.stop()
                        self.vel_logger.stop()
                        return False
                    
                    # Estrategia de recuperación: retroceder un poco después de
                    # una colisión para dar espacio al robot antes de continuar
                    print("[INFO] Retrocediendo...")
                    await self.robot.set_wheel_speeds(-10, -10)
                    await self.robot.wait(1.0)
                    await self.robot.set_wheel_speeds(0, 0)
                    await self.robot.wait(0.5)
                    continue
                
                # Saturar las velocidades dentro de los límites seguros del robot
                # Las velocidades ya vienen combinadas del potencial, así que solo
                # necesitamos asegurar que no excedan los límites físicos
                v_left, v_right = saturate_wheel_speeds(v_left, v_right)
                
                # ========== CONTROL DE LEDs Y SONIDO SEGÚN ESTADO ==========
                # Sistema de LEDs:
                # - VERDE: Inicio (ya establecido al principio)
                # - AZUL: Navegando sin obstáculos
                # - NARANJA: Obstáculo detectado (con pitido)
                # - CYAN: Esquivando obstáculo activamente
                # - VERDE: Meta alcanzada
                
                # Obtener información de obstáculos y nivel de seguridad
                num_obstacles = info.get('num_obstacles', 0)
                safety_level = info.get('safety_level', 'CLEAR')
                max_ir_front = info.get('max_ir_front', 0)
                
                # Determinar el estado actual del robot
                if num_obstacles > 0 and max_ir_front >= config.IR_THRESHOLD_CAUTION:
                    # Hay obstáculos detectados dentro del rango de influencia
                    if max_ir_front >= config.IR_THRESHOLD_WARNING:
                        # ESQUIVANDO: Obstáculo cerca, maniobra activa
                        if self.current_led_color != 'cyan':
                            await self.robot.set_lights_rgb(0, 255, 255)  # CYAN
                            self.current_led_color = 'cyan'
                    else:
                        # OBSTÁCULO DETECTADO: Primera detección
                        if self.current_led_color != 'orange':
                            await self.robot.set_lights_rgb(255, 165, 0)  # NARANJA
                            self.current_led_color = 'orange'
                            # Emitir pitido solo cuando cambia a naranja (primera detección)
                            if not self.obstacle_detected:
                                await self.robot.play_note(440, 0.2)  # La (440Hz) por 0.2 segundos
                                self.obstacle_detected = True
                else:
                    # Sin obstáculos cercanos: navegación normal
                    if self.current_led_color != 'blue':
                        await self.robot.set_lights_rgb(0, 0, 255)  # AZUL
                        self.current_led_color = 'blue'
                        # Resetear flag de obstáculo cuando vuelve a navegación normal
                        self.obstacle_detected = False
                
                # Mostrar información de debug si está habilitado
                # Incluimos información sobre obstáculos detectados y fuerzas repulsivas
                if self.debug and iteration % 10 == 0:
                    num_obs = info.get('num_obstacles', 0)
                    fx_rep = info.get('fx_repulsive', 0.0)
                    fy_rep = info.get('fy_repulsive', 0.0)
                    print(f"[{iteration:04d}] d={distance:5.1f} obs={num_obs} "
                          f"F_rep=({fx_rep:6.1f},{fy_rep:6.1f}) "
                          f"v_l={v_left:5.1f} v_r={v_right:5.1f}")
                
                # Enviar comandos de velocidad a las ruedas del robot
                await self.robot.set_wheel_speeds(v_left, v_right)
                
                # Esperar el período de control antes de la siguiente iteración
                await self.robot.wait(config.CONTROL_DT)
        
        except Exception as e:
            # Manejo de errores durante la navegación con información detallada
            print(f"\n[ERROR] Error durante navegacion: {e}")
            import traceback
            traceback.print_exc()
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
    Función principal que inicializa y ejecuta el sistema de navegación combinada.
    
    Esta función es similar a la de PRM01_P01.py pero incluye argumentos adicionales
    para configurar los parámetros del potencial repulsivo. Parseamos los argumentos
    de línea de comandos, cargamos los puntos de navegación, establecemos la
    conexión Bluetooth, y configuramos el manejo de interrupciones.
    
    Los argumentos adicionales permiten ajustar la ganancia repulsiva y la distancia
    de influencia, lo que nos permite experimentar con diferentes configuraciones
    según las características del entorno de navegación.
    """
    
    # Configurar el parser de argumentos de línea de comandos
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
        default=f"data/{config.POINTS_FILE}",
        help=f"Archivo JSON con puntos (default: data/{config.POINTS_FILE})"
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
    
    # Parsear los argumentos proporcionados
    args = parser.parse_args()
    
    # Cargar los puntos de navegación desde el archivo JSON
    q_i, q_f = load_points(args.points)
    
    # Mostrar información de la misión antes de iniciar, incluyendo parámetros
    # del potencial repulsivo
    print_mission_info(q_i, q_f, args.robot, args.potential, 
                      k_rep=args.k_rep, d_influence=args.d_influence)
    
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
        
        # Crear la instancia del navegador con los parámetros configurados,
        # incluyendo los parámetros del potencial repulsivo
        navigator = CombinedPotentialNavigator(
            robot, q_f, 
            potential_type=args.potential,
            k_rep=args.k_rep,
            d_influence=args.d_influence,
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