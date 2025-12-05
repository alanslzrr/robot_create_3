#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script principal de navegación con campos de potencial combinados - Parte 02

===============================================================================
INFORMACIÓN DEL PROYECTO
===============================================================================

Autores:
    - Alan Ariel Salazar
    - Yago Ramos Sánchez

Institución:
    Universidad Intercontinental de la Empresa (UIE)

Profesor:
    Eladio Dapena

Asignatura:
    Robots Autónomos

Fecha de Finalización:
    6 de noviembre de 2025

Robot SDK:
    irobot-edu-sdk

Para revisar el resto de los archivos consultar el siguiente repo: https://github.com/alanslzrr/robot_create_3/tree/main/PL4

===============================================================================
OBJETIVO GENERAL
===============================================================================

Desarrollar un sistema de navegación autónoma para el robot iRobot Create 3 que
combine campos de potencial atractivo y repulsivo, permitiendo al robot navegar
desde una posición inicial hasta un objetivo mientras evita obstáculos detectados
mediante sensores infrarrojos en tiempo real.

Este script extiende la funcionalidad de la Parte 01 incorporando evasión
inteligente de obstáculos mediante fuerzas repulsivas calculadas a partir de
las lecturas de los siete sensores IR del robot, logrando una navegación más
robusta y segura en entornos con obstáculos.

===============================================================================
OBJETIVOS ESPECÍFICOS
===============================================================================

1. Implementar un sistema de navegación que combine campos de potencial atractivo
   y repulsivo para lograr navegación autónoma con evasión de obstáculos

2. Utilizar los sensores infrarrojos del robot para detectar obstáculos en tiempo
   real y calcular fuerzas repulsivas apropiadas basadas en un modelo físico

3. Convertir las lecturas de sensores IR en posiciones estimadas de obstáculos
   utilizando un modelo físico basado en la relación inversa al cuadrado y
   compensación por ángulo del sensor

4. Combinar vectorialmente las fuerzas atractivas y repulsivas para generar una
   dirección de movimiento resultante que evite colisiones manteniendo el objetivo
   de llegar a la meta

5. Implementar un sistema de control de velocidad dinámico que ajuste la velocidad
   máxima permitida según la proximidad de obstáculos detectados, garantizando
   tiempo suficiente de reacción y frenado

6. Detectar espacios navegables (gaps) entre obstáculos para permitir que el robot
   pase por pasillos estrechos sin detenerse innecesariamente

7. Mantener la capacidad de análisis comparativo registrando datos adicionales
   sobre fuerzas repulsivas, obstáculos detectados y niveles de seguridad en
   archivos CSV para análisis posterior

===============================================================================
CONFIGURACIÓN
===============================================================================

El script está configurado de manera similar a la Parte 01, pero incorpora
parámetros adicionales para el potencial repulsivo. Los principales parámetros
configurables incluyen:

- Ganancias de potencial atractivo (iguales a la Parte 01: lineal, cuadrática,
  cónica y exponencial)
- Ganancia repulsiva (K_REPULSIVE) que controla la intensidad de la evasión
- Distancia de influencia (D_INFLUENCE) que define el rango efectivo de los
  campos repulsivos (60 cm por defecto)
- Sistema de umbrales escalonados para control dinámico de velocidad según
  proximidad de obstáculos (emergencia, crítico, advertencia, precaución)

El script utiliza los mismos archivos de configuración y puntos de navegación
que la Parte 01 (data/points.json), pero ahora también acepta argumentos de
línea de comandos para ajustar los parámetros del potencial repulsivo,
permitiendo experimentar con diferentes configuraciones según las características
del entorno.

===============================================================================
DIFERENCIAS CON LA PARTE 01
===============================================================================

La principal diferencia con respecto a PRM01_P01.py es que utilizamos la función
combined_potential_speeds() en lugar de attractive_wheel_speeds(). Esta función
no solo calcula la fuerza atractiva hacia el objetivo, sino que también:

- Lee las lecturas de los siete sensores infrarrojos del robot en cada iteración
- Normaliza las lecturas según la sensibilidad específica de cada sensor
- Estima las posiciones de obstáculos basándose en un modelo físico mejorado
  que compensa por el ángulo del sensor
- Calcula fuerzas repulsivas para cada obstáculo detectado usando el concepto
  de clearance (distancia libre después del radio del robot)
- Detecta espacios navegables (gaps) entre obstáculos para permitir paso por
  pasillos estrechos
- Combina vectorialmente todas las fuerzas para obtener la dirección resultante
- Aplica un sistema de límites dinámicos de velocidad según el clearance disponible
  y la proximidad de obstáculos detectados

El sistema está diseñado para mantener al robot avanzando hacia el objetivo
mientras evita obstáculos de forma inteligente, en lugar de detenerse completamente
cuando detecta un obstáculo cercano. Esto permite una navegación más fluida y
eficiente en entornos con múltiples obstáculos.
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
    
    Esta función lee el archivo JSON que contiene las coordenadas del punto
    inicial (q_i) y el punto final (q_f). Realizamos validaciones exhaustivas
    para asegurar que el archivo existe, tiene formato JSON válido, y contiene
    la estructura esperada con las claves necesarias.
    
    También verificamos que los puntos no estén demasiado cerca entre sí, ya
    que esto podría causar problemas en la navegación o hacer que el robot
    termine inmediatamente después de iniciar.
    
    Args:
        filename: Ruta al archivo JSON con los puntos de navegación
    
    Returns:
        tuple: Tupla con (q_i, q_f) donde:
            - q_i: Tupla (x, y, theta) con posición y orientación inicial en cm y grados
            - q_f: Tupla (x, y) con posición final en cm
    
    Raises:
        SystemExit: Si el archivo no existe, tiene formato inválido, o no
                    contiene la estructura esperada (q_i y q_f)
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
    
    Esta función muestra un resumen completo de todos los parámetros de la misión
    antes de iniciar la navegación. Calculamos la distancia a recorrer y el
    ángulo hacia la meta, seleccionamos la ganancia lineal apropiada según el
    tipo de potencial elegido, y mostramos tanto los parámetros atractivos como
    los repulsivos para que el usuario pueda verificar la configuración antes
    de comenzar.
    
    La información mostrada incluye:
    - Puntos inicial y final con sus coordenadas
    - Distancia total a recorrer y ángulo hacia la meta
    - Parámetros de control (ganancias atractiva, angular y repulsiva)
    - Distancia de influencia del campo repulsivo
    - Velocidad máxima y tolerancia de llegada
    
    Args:
        q_i: Tupla (x, y, theta) con posición y orientación inicial en cm y grados
        q_f: Tupla (x, y) con posición final en cm
        robot_name: Nombre Bluetooth del robot a conectar (nuestro robot del grupo 1)
        potential_type: Tipo de función de potencial atractivo ('linear', 'quadratic',
                        'conic', 'exponential')
        k_rep: Ganancia repulsiva en cm/s² (usa config.K_REPULSIVE si es None)
        d_influence: Distancia de influencia repulsiva en cm (usa config.D_INFLUENCE
                     si es None)
    """
    # Calcular distancia y ángulo hacia la meta
    distance = math.hypot(q_f[0] - q_i[0], q_f[1] - q_i[1])
    angle = math.degrees(math.atan2(q_f[1] - q_i[1], q_f[0] - q_i[0]))
    
    # Seleccionamos la ganancia lineal apropiada según el tipo de potencial elegido
    # Cada función de potencial tiene características diferentes de escala, por lo
    # que requiere una ganancia específica para lograr comportamientos similares
    if potential_type == 'linear':
        k_lin = config.K_LINEAR
    elif potential_type == 'quadratic':
        k_lin = config.K_QUADRATIC
    elif potential_type == 'conic':
        k_lin = config.K_CONIC
    elif potential_type == 'exponential':
        k_lin = config.K_EXPONENTIAL
    else:
        k_lin = config.K_LINEAR  # Por defecto usamos lineal si el tipo no es reconocido
    
    # Usamos los valores por defecto de configuración si no se especificaron
    # Esto permite que el usuario omita estos parámetros y use los valores calibrados
    if k_rep is None:
        k_rep = config.K_REPULSIVE
    if d_influence is None:
        d_influence = config.D_INFLUENCE
    
    # Mostramos toda la información formateada incluyendo parámetros repulsivos
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
    campos de potencial combinados. A diferencia de AttractiveFieldNavigator de
    la Parte 01, esta clase integra tanto las fuerzas atractivas hacia el
    objetivo como las fuerzas repulsivas de los obstáculos detectados mediante
    sensores IR.
    
    El sistema funciona leyendo continuamente los siete sensores infrarrojos
    del robot en cada iteración del bucle de control (20 Hz). Estimamos las
    posiciones de obstáculos basándonos en un modelo físico mejorado que relaciona
    la intensidad de la señal IR con la distancia, compensando por el ángulo del
    sensor. Luego calculamos fuerzas repulsivas que se combinan vectorialmente
    con la fuerza atractiva hacia el objetivo.
    
    El resultado es una navegación que se ajusta dinámicamente para evitar
    colisiones mientras mantiene el objetivo de llegar a la meta. El robot
    nunca se detiene completamente cuando detecta obstáculos, sino que modifica
    su trayectoria para esquivarlos de forma inteligente.
    
    Características principales:
    - Detección de obstáculos en tiempo real mediante 7 sensores IR
    - Cálculo de fuerzas repulsivas basado en clearance (distancia libre)
    - Detección de espacios navegables (gaps) entre obstáculos
    - Control dinámico de velocidad según proximidad de obstáculos
    - Sistema de escape de trampas en C (mínimos locales)
    - Transformación de coordenadas para trabajar en sistema mundial
    """
    
    def __init__(self, robot, q_initial, q_goal, potential_type='linear', k_rep=None, d_influence=None, debug=False):
        """
        Inicializa el navegador con los parámetros de configuración.
        
        Creamos las instancias de los loggers de sensores y velocidades, y
        almacenamos todas las referencias necesarias. Además de los parámetros
        básicos de navegación, también almacenamos los parámetros específicos
        del potencial repulsivo que pueden ser ajustados mediante argumentos
        de línea de comandos para experimentar con diferentes configuraciones.
        
        También configuramos el sistema de transformación de coordenadas para
        convertir las lecturas de odometría del robot (que empiezan en 0,0 después
        del reset) al sistema mundial especificado en points.json.
        
        Args:
            robot: Instancia del robot Create3 conectado (nuestro robot del grupo 1)
            q_initial: Tupla (x, y, theta) con posición y orientación inicial en cm y grados
            q_goal: Tupla (x, y) con coordenadas del objetivo en cm
            potential_type: Tipo de función de potencial atractivo ('linear', 'quadratic',
                           'conic', 'exponential')
            k_rep: Ganancia repulsiva en cm/s² (usa config.K_REPULSIVE si es None)
            d_influence: Distancia de influencia repulsiva en cm (usa config.D_INFLUENCE
                        si es None)
            debug: Si es True, muestra información detallada cada 10 iteraciones del bucle
        """
        self.robot = robot
        self.q_initial = q_initial  # Guardar posición y orientación inicial
        self.q_goal = q_goal
        self.potential_type = potential_type
        self.k_rep = k_rep or config.K_REPULSIVE
        self.d_influence = d_influence or config.D_INFLUENCE
        self.debug = debug
        self.vel_logger = VelocityLogger(f"{potential_type}_combined")
        self.running = False
        self.current_led_color = None  # Para rastrear el color actual del LED
        self.obstacle_detected = False  # Para rastrear si ya se detectó obstáculo (para sonido)
        
        # TRANSFORMACIÓN DE COORDENADAS:
        # El robot internamente usa odometría que empieza en (0,0) después de
        # reset_navigation(), pero nosotros queremos trabajar en un sistema de
        # coordenadas donde la posición inicial q_i está en (x_i, y_i) especificada
        # en points.json.
        # 
        # IMPORTANTE: No solo necesitamos un offset de posición, sino una
        # TRANSFORMACIÓN completa porque el robot puede empezar con cualquier
        # orientación. Guardamos los parámetros de transformación para aplicarlos
        # a todas las lecturas de odometría durante la navegación.
        self.q_initial = q_initial  # Guardamos para usar en transformaciones
        self.position_offset_x = q_initial[0]  # Offset en X del sistema mundial
        self.position_offset_y = q_initial[1]  # Offset en Y del sistema mundial
        self.initial_heading = q_initial[2]     # Heading deseado inicial
        
        # Variables para la transformación de coordenadas
        # Estas se inicializarán después del reset_navigation() cuando conozcamos
        # el heading real del robot
        self.odometry_to_world_rotation = 0.0  # Rotación del sistema de odometría al mundo (radianes)
        self.reset_heading = 0.0  # Heading real del robot al hacer reset (grados)
        
        # Creamos el SensorLogger con los offsets de posición para que muestre
        # posiciones corregidas en el sistema mundial. El heading_offset se
        # calculará después del reset cuando conozcamos el heading real.
        self.logger = SensorLogger(robot, 
                                   position_offset_x=self.position_offset_x,
                                   position_offset_y=self.position_offset_y,
                                   heading_offset=0)  # Se actualizará después del reset
        
        print(f"[INFO] Sistema de coordenadas configurado:")
        print(f"       Posicion inicial deseada: ({self.position_offset_x:.1f}, {self.position_offset_y:.1f}) cm")
        print(f"       Heading inicial deseado: {self.initial_heading:.1f}°")
        
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
        # Reseteamos la odometría del robot al inicio de la navegación
        # IMPORTANTE: reset_navigation() establece la posición interna del robot
        # a (0, 0, heading_actual), donde heading_actual es la orientación real
        # del robot en ese momento. Esto significa que el sistema de coordenadas
        # interno del robot empieza desde cero, pero con la orientación actual.
        await self.robot.reset_navigation()
        
        # Obtenemos el heading real inicial del robot después del reset
        # Este es el ángulo al que realmente está apuntando el robot físicamente
        pos_initial = await self.robot.get_position()
        self.reset_heading = pos_initial.heading  # Guardamos para usar en transformaciones
        desired_heading = self.q_initial[2]  # Heading deseado según points.json
        
        # Calculamos el ángulo de rotación entre el sistema de odometría y el mundo
        # Este ángulo se usa para rotar las coordenadas de odometría al sistema mundial
        # Si el robot está apuntando a -134.7° pero queremos que esté en 0°, necesitamos
        # rotar las coordenadas por la diferencia
        self.odometry_to_world_rotation = math.radians(desired_heading - self.reset_heading)
        
        # Calculamos el offset angular para mostrar en los logs y transformaciones
        # Este offset se suma a todas las lecturas de heading para convertirlas al
        # sistema mundial
        self.heading_offset = desired_heading - self.reset_heading
        
        # Actualizamos el heading_offset en el logger para que muestre ángulos
        # corregidos en el sistema mundial en lugar del sistema de odometría
        self.logger.heading_offset = self.heading_offset
        
        print(f"[INFO] Configuracion de odometria:")
        print(f"       Posicion inicial deseada (points.json): ({self.q_initial[0]:.1f}, {self.q_initial[1]:.1f}, {desired_heading:.1f}°)")
        print(f"       Heading real del robot al inicio: {self.reset_heading:.1f}°")
        print(f"       Rotacion del sistema de coordenadas: {math.degrees(self.odometry_to_world_rotation):.1f}°")
        print(f"       Posicion final objetivo: ({self.q_goal[0]:.1f}, {self.q_goal[1]:.1f})")
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
        MAX_COLLISIONS = 5  # Aumentado de 3 a 5 para dar más oportunidades de navegación
        
        try:
            while self.running:
                iteration += 1
                
                # Leer el estado actual del robot
                pos = await self.robot.get_position()
                
                # VALIDACIÓN: A veces get_position() puede devolver None
                if pos is None:
                    print("[WARNING] get_position() devolvió None, reintentando...")
                    await self.robot.wait(0.05)
                    continue
                
                # TRANSFORMACIÓN DE COORDENADAS desde odometría a sistema mundial
                # La odometría del robot está en su propio sistema de referencia que empieza
                # en (0, 0) después del reset. Necesitamos transformar estas coordenadas
                # al sistema mundial especificado en points.json mediante una rotación y
                # una traslación.
                
                # Paso 1: Obtenemos la posición en el sistema de odometría del robot
                # Estas coordenadas están en el sistema local del robot que empieza en (0,0)
                odom_x = pos.x
                odom_y = pos.y
                
                # Paso 2: Rotamos las coordenadas según la orientación inicial del robot
                # Si el robot empezó apuntando hacia -134.7°, su eje X interno apunta en esa
                # dirección. Necesitamos rotar el vector (odom_x, odom_y) para llevarlo al
                # sistema mundial donde el eje X apunta hacia el este (0°).
                # Usamos una matriz de rotación 2D estándar
                cos_rot = math.cos(self.odometry_to_world_rotation)
                sin_rot = math.sin(self.odometry_to_world_rotation)
                
                rotated_x = odom_x * cos_rot - odom_y * sin_rot
                rotated_y = odom_x * sin_rot + odom_y * cos_rot
                
                # Paso 3: Trasladamos al punto inicial deseado sumando los offsets
                # Ahora las coordenadas están rotadas correctamente, solo necesitamos
                # moverlas al punto inicial especificado en points.json
                actual_x = rotated_x + self.position_offset_x
                actual_y = rotated_y + self.position_offset_y
                
                # Aplicamos el offset de heading para convertir el ángulo al sistema mundial
                # El heading del robot también necesita ser corregido para que 0° corresponda
                # a la dirección deseada según points.json
                actual_heading = pos.heading + self.heading_offset
                
                # Normalizamos el heading al rango [-180, 180] para mantener consistencia
                # Esto evita valores como 185° que deberían ser -175°
                while actual_heading > 180:
                    actual_heading -= 360
                while actual_heading <= -180:
                    actual_heading += 360
                
                # Posición completa en nuestro sistema de coordenadas mundial
                q = (actual_x, actual_y, actual_heading)
                
                # CALCULAR DISTANCIA AL OBJETIVO usando la posición corregida
                dx = self.q_goal[0] - actual_x
                dy = self.q_goal[1] - actual_y
                distance = math.hypot(dx, dy)
                
                # DETENCIÓN INMEDIATA si estamos en el objetivo
                # Esto evita que el robot gire sobre su eje cuando llega
                # Aumentamos ligeramente la tolerancia porque la odometría tiene drift
                TOLERANCE_CM = 10.0  # 10 cm en vez de 5 cm para convergencia más fácil
                if distance < TOLERANCE_CM:
                    print(f"\n[SUCCESS] Meta alcanzada! Distancia: {distance:.2f} cm")
                    print(f"           Posicion final: x={actual_x:.1f}, y={actual_y:.1f}, theta={actual_heading:.1f} deg")
                    print(f"           Objetivo: x={self.q_goal[0]:.1f}, y={self.q_goal[1]:.1f}")
                    await self.robot.set_wheel_speeds(0, 0)
                    await self.robot.set_lights_rgb(0, 255, 0)  # LED VERDE
                    await self.robot.play_note(80, 0.2)
                    self.logger.stop()
                    self.vel_logger.stop()
                    self.running = False
                    return True
                
                # Leemos los sensores IR para detección de obstáculos en tiempo real
                # Estos siete sensores nos permiten detectar obstáculos alrededor del
                # frente del robot y calcular fuerzas repulsivas apropiadas
                ir_prox = await self.robot.get_ir_proximity()
                ir_sensors = ir_prox.sensors if hasattr(ir_prox, 'sensors') else []
                
                # También leemos los bumpers para detectar colisiones físicas
                # Los bumpers solo se activan cuando ya hay contacto, así que son
                # nuestra última línea de defensa
                bumpers = await self.robot.get_bumpers()
                
                # DEBUG: Mostramos los valores que estamos usando para navegación
                # Solo en las primeras 3 iteraciones para verificar que todo funciona
                if iteration <= 3:
                    print(f"\n[DEBUG iter {iteration}] q={q}, q_goal={self.q_goal}")
                    print(f"[DEBUG iter {iteration}] dx={dx:.2f}, dy={dy:.2f}, distance={distance:.2f}")
                
                # Calculamos las velocidades usando potencial COMBINADO (atractivo + repulsivo)
                # Esta función toma en cuenta las lecturas IR para calcular obstáculos
                # y generar fuerzas repulsivas que modifican la trayectoria. El robot
                # siempre intenta avanzar hacia el objetivo, pero ajusta su dirección
                # para evitar colisiones.
                v_left, v_right, dist_from_func, info = combined_potential_speeds(
                    q, self.q_goal, 
                    ir_sensors=ir_sensors,
                    k_rep=self.k_rep,
                    d_influence=self.d_influence,
                    potential_type=self.potential_type
                )
                
                # IMPORTANTE: No aplicamos reducción de velocidad adicional por IR aquí
                # porque el potencial repulsivo ya maneja la evasión de forma inteligente.
                # El robot debe seguir avanzando y esquivando obstáculos, no detenerse
                # completamente cuando detecta obstáculos cercanos. Esto permite una
                # navegación más fluida y efectiva en entornos con múltiples obstáculos.
                
                # Registramos los datos de esta iteración en el archivo CSV
                # Incluimos información adicional sobre fuerzas repulsivas, obstáculos
                # detectados y nivel de seguridad para análisis posterior
                self.vel_logger.log(
                    {'x': pos.x, 'y': pos.y, 'theta': pos.heading},
                    distance, v_left, v_right, info
                )
                
                # Ya verificamos la distancia arriba y nos detuvimos si llegamos a la meta
                # Aquí continuamos con el manejo de colisiones y el envío de velocidades
                
                # Manejo de emergencias: colisión física detectada por bumpers
                # Los bumpers solo se activan cuando ya hay contacto físico, así que
                # esto indica que nuestras fuerzas repulsivas no fueron suficientes
                if emergency_stop_needed(bumpers):
                    collision_count += 1
                    await self.robot.set_wheel_speeds(0, 0)  # Detenemos inmediatamente
                    print(f"\n[COLLISION] Colision {collision_count}/{MAX_COLLISIONS} detectada")
                    
                    # Si excedemos el número máximo de colisiones permitidas, abortamos
                    # la misión porque probablemente el camino está completamente bloqueado
                    if collision_count >= MAX_COLLISIONS:
                        print(f"[ERROR] Camino bloqueado - demasiadas colisiones")
                        self.logger.stop()
                        self.vel_logger.stop()
                        return False
                    
                    # Estrategia de recuperación: retrocedemos un poco después de una
                    # colisión para dar espacio al robot antes de continuar. Esto permite
                    # que el robot se reposicione y encuentre una mejor trayectoria.
                    print("[INFO] Retrocediendo...")
                    await self.robot.set_wheel_speeds(-10, -10)  # Retrocedemos a velocidad moderada
                    await self.robot.wait(1.0)  # Retrocedemos por 1 segundo
                    await self.robot.set_wheel_speeds(0, 0)  # Nos detenemos
                    await self.robot.wait(0.5)  # Esperamos medio segundo antes de continuar
                    continue  # Volvemos al inicio del bucle para recalcular trayectoria
                
                # Saturaremos las velocidades dentro de los límites seguros del robot
                # Las velocidades ya vienen combinadas del potencial, así que solo
                # necesitamos asegurar que no excedan los límites físicos del hardware
                # Esto protege los motores de comandos excesivos
                v_left, v_right = saturate_wheel_speeds(v_left, v_right)
                
                # ========== CONTROL DE LEDs Y SONIDO SEGÚN ESTADO ==========
                # Sistema de LEDs para feedback visual del estado del robot:
                # - VERDE: Inicio (ya establecido al principio) o meta alcanzada
                # - AZUL: Navegando sin obstáculos cercanos
                # - NARANJA: Obstáculo detectado (con pitido de alerta)
                # - CYAN: Esquivando obstáculo activamente (maniobra en curso)
                
                # Obtenemos información de obstáculos y nivel de seguridad del diccionario
                # de información retornado por combined_potential_speeds()
                num_obstacles = info.get('num_obstacles', 0)
                safety_level = info.get('safety_level', 'CLEAR')
                max_ir_all = info.get('max_ir_all', 0)
                
                # Determinamos el estado actual del robot y cambiamos el LED apropiadamente
                if num_obstacles > 0 and max_ir_all >= config.IR_THRESHOLD_CAUTION:
                    # Hay obstáculos detectados dentro del rango de influencia
                    if max_ir_all >= config.IR_THRESHOLD_WARNING:
                        # ESQUIVANDO: Obstáculo cerca, maniobra activa de evasión
                        # El robot está modificando su trayectoria para evitar el obstáculo
                        if self.current_led_color != 'cyan':
                            await self.robot.set_lights_rgb(0, 255, 255)  # CYAN
                            self.current_led_color = 'cyan'
                    else:
                        # OBSTÁCULO DETECTADO: Primera detección de un obstáculo
                        # El robot acaba de detectar un obstáculo pero aún no está muy cerca
                        if self.current_led_color != 'orange':
                            await self.robot.set_lights_rgb(255, 165, 0)  # NARANJA
                            self.current_led_color = 'orange'
                            # Emitimos un pitido solo cuando cambia a naranja (primera detección)
                            # para alertar visual y auditivamente
                            if not self.obstacle_detected:
                                await self.robot.play_note(440, 0.2)  # La (440Hz) por 0.2 segundos
                                self.obstacle_detected = True
                else:
                    # Sin obstáculos cercanos: navegación normal hacia el objetivo
                    if self.current_led_color != 'blue':
                        await self.robot.set_lights_rgb(0, 0, 255)  # AZUL
                        self.current_led_color = 'blue'
                        # Reseteamos el flag de obstáculo cuando vuelve a navegación normal
                        # para que pueda sonar de nuevo si detecta otro obstáculo más adelante
                        self.obstacle_detected = False
                
                # Mostramos información de debug si está habilitado
                # Incluimos información sobre obstáculos detectados y fuerzas repulsivas
                # para poder analizar el comportamiento del sistema durante el desarrollo
                if self.debug and iteration % 10 == 0:
                    num_obs = info.get('num_obstacles', 0)
                    fx_rep = info.get('fx_repulsive', 0.0)
                    fy_rep = info.get('fy_repulsive', 0.0)
                    print(f"[{iteration:04d}] d={distance:5.1f} obs={num_obs} "
                          f"F_rep=({fx_rep:6.1f},{fy_rep:6.1f}) "
                          f"v_l={v_left:5.1f} v_r={v_right:5.1f}")
                
                # Enviamos los comandos de velocidad a las ruedas del robot
                # Estas velocidades ya están saturadas y listas para ejecutar
                await self.robot.set_wheel_speeds(v_left, v_right)
                
                # Esperamos el período de control antes de la siguiente iteración
                # Esto mantiene el bucle de control a 20 Hz (50 ms por iteración)
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
    
    # Establecemos la conexión Bluetooth con nuestro robot del grupo 1
    # El nombre por defecto es "C3_UIEC_Grupo1" según config.py
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
        # incluyendo la posición inicial completa (x, y, theta) y los
        # parámetros del potencial repulsivo
        navigator = CombinedPotentialNavigator(
            robot, q_i, q_f,  # Pasar AMBOS q_i (con theta) y q_f
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