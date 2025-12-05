"""
================================================================================
PRÁCTICA DE LABORATORIO 02 - ROBOTS AUTÓNOMOS
ETAPA 04: LUGAR DE FINALIZACIÓN
================================================================================

INFORMACIÓN BÁSICA:
- Autores: Yago Ramos - Salazar Alan
- Fecha de finalización: 7 de octubre de 2025
- Institución: UIE

OBJETIVO:
- Continuar desplazándose por el entorno utilizando el mismo procedimiento
- Repetir el modelo de Etapa 02 hasta encontrar un lugar donde AMBOS caminos
  estén ocupados por obstáculos (final de la Ronda Aleatoria)
- Implementar navegación autónoma continua con detección de punto final

COMPORTAMIENTO ESPERADO:
1. Ejecutar Etapa 01 completa (reset, avance, parada a 15 cm)
2. Entrar en bucle continuo de navegación:
   - Inspeccionar laterales con luz AMARILLA
   - Decidir giro basado en sensores laterales
   - Girar 90° hacia lado más libre
   - Avanzar hasta obstáculo con señalización
3. Terminar cuando AMBOS lados estén bloqueados (sin salida)
4. Reportar distancia total recorrida desde inicio

SENSORES UTILIZADOS:
- IR Proximity Sensor (índice 3): Detección frontal de obstáculos (~15 cm)
- IR Proximity Sensors (índices 0,1): Sensores laterales izquierdos
- IR Proximity Sensors (índices 5,6): Sensores laterales derechos
- IR_DIR_THRESHOLD = 200: Umbral para decisión de giro

CONDICIÓN DE TERMINACIÓN:
- El robot debe continuar hasta que en la inspección lateral:
  - Izquierda > IR_DIR_THRESHOLD (bloqueado)
  - Y Derecha > IR_DIR_THRESHOLD (bloqueado)
- Esto indica que se ha llegado al final de la Ronda Aleatoria

LÓGICA DE BUCLE:
- while True: Continúa indefinidamente hasta encontrar sin salida
- En cada iteración: inspección → decisión → giro → avance → detección
- Solo termina cuando ambos lados están bloqueados simultáneamente
"""

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# ==============================================
# CONFIGURACIÓN Y PARÁMETROS
# ==============================================
# Conexión Bluetooth con el robot Create3 del laboratorio
robot = Create3(Bluetooth("C3_UIEC_Grupo1"))

# Umbrales de sensores IR
IR_OBS_THRESHOLD = 120   # ~15 cm para detección frontal de obstáculos
IR_DIR_THRESHOLD = 200   # Umbral para decisión de giro (CRÍTICO: no cambiar)


# ==============================================
# FUNCIONES AUXILIARES
# ==============================================
async def detectar_obstaculo(robot, umbral: int = IR_OBS_THRESHOLD) -> bool:
    """
    FUNCIÓN DE DETECCIÓN DE OBSTÁCULOS FRONTALES
    
    Monitorea continuamente el sensor IR frontal central hasta detectar
    un obstáculo a la distancia especificada por el umbral.
    
    Parámetros:
        robot: Instancia del robot Create3
        umbral: Valor umbral para detección (default: IR_OBS_THRESHOLD = 120)
    
    Retorna:
        bool: True cuando se detecta obstáculo y debe detenerse
    """
    while True:
        # Obtener lectura actual de todos los sensores IR
        ir = (await robot.get_ir_proximity()).sensors
        
        # Verificar que el sensor frontal central esté disponible y supere umbral
        if len(ir) > 3 and ir[3] > umbral:
            return True  # Obstáculo detectado a ~15 cm


# ==============================================
# EJECUCIÓN PRINCIPAL - ETAPA 04
# ==============================================
@event(robot.when_play)
async def play(robot):
    """
    SECUENCIA PRINCIPAL DE LA ETAPA 04
    
    Esta función ejecuta la secuencia completa de la Etapa 04 siguiendo
    exactamente las especificaciones del laboratorio.
    
    FLUJO DE EJECUCIÓN:
    1. Ejecutar Etapa 01 completa (reset, avance, parada a 15 cm)
    2. Entrar en bucle continuo de navegación autónoma
    3. En cada iteración: inspección → decisión → giro → avance
    4. Terminar cuando AMBOS lados estén bloqueados (sin salida)
    5. Reportar distancia total recorrida desde inicio
    
    CONDICIÓN DE TERMINACIÓN:
    - while True: Continúa hasta que ambos lados estén bloqueados
    - Solo termina cuando encuentra el final de la Ronda Aleatoria
    """
    print("=" * 50)
    print("ETAPA 04: LUGAR DE FINALIZACIÓN")
    print("=" * 50)

    # ============================================
    # ETAPA 01 COMPLETA - PASOS A-G
    # ============================================
    print("→ Ejecutando Etapa 01 completa...")
    
    # a) Reset de navegación
    await robot.reset_navigation()
    print("✓ Reset de navegación completado")

    # b) Señal inicial: luz AZUL + sonido
    await robot.set_lights_on_rgb(0, 0, 255)      # RGB: Azul
    await robot.play_note(440, 0.5)               # Nota A4 por 0.5 segundos
    print("✓ Señal inicial: Luz AZUL + sonido")

    # c) Capturar posición inicial para cálculo de distancia total
    pos_inicial = await robot.get_position()
    x0, y0 = pos_inicial.x, pos_inicial.y
    print(f"✓ Posición inicial: Pose({x0:.2f}, {y0:.2f}, {pos_inicial.heading:.1f}°)")

    # d) Velocidad 5 cm/s
    await robot.set_lights_on_rgb(0, 0, 255)      # AZUL durante navegación
    await robot.set_wheel_speeds(5, 5)
    print("✓ Velocidad establecida: 5 cm/s")

    # e) Avanzar hasta obstáculo a ~15 cm
    print("→ Avanzando hasta primer obstáculo...")
    if await detectar_obstaculo(robot):
        await robot.set_wheel_speeds(0, 0)
        await robot.wait(0.2)
        print("✓ Primer obstáculo detectado - Robot detenido")

    # f) Señal ROJA + sonido (obstáculo detectado)
    await robot.set_lights_on_rgb(255, 0, 0)      # RGB: Rojo
    await robot.play_note(440, 0.5)
    print("✓ Señal de detección: Luz ROJA + sonido")

    # g) Señal VERDE + sonido (fin Etapa 01)
    await robot.set_lights_on_rgb(0, 255, 0)      # RGB: Verde
    await robot.play_note(523, 0.5)
    print("✓ Fin Etapa 01: Luz VERDE + sonido")

    # ============================================
    # BUCLE CONTINUO DE NAVEGACIÓN AUTÓNOMA
    # ============================================
    print("\n→ Iniciando navegación autónoma continua...")
    print("→ Buscando lugar de finalización (ambos lados bloqueados)...")

    while True:
        # ============================================
        # INSPECCIÓN LATERAL
        # ============================================
        # d) Luz AMARILLA durante inspección
        await robot.set_lights_on_rgb(255, 255, 0)    # RGB: Amarillo
        print("→ Inspeccionando laterales...")

        # Leer sensores laterales: izquierda = (0,1), derecha = (5,6)
        ir = (await robot.get_ir_proximity()).sensors
        izq_vals = [ir[i] for i in (0, 1) if i < len(ir)]
        der_vals = [ir[i] for i in (5, 6) if i < len(ir)]
        
        # Métrica por lado: valor máximo (peor caso). Menor => más libre
        izquierda = max(izq_vals) if izq_vals else 0
        derecha = max(der_vals) if der_vals else 0

        # Mostrar valores en consola
        print(f"✓ Lectura lateral: Izq={izquierda}, Der={derecha} (Umbral={IR_DIR_THRESHOLD})")

        # ============================================
        # CONDICIÓN DE TERMINACIÓN
        # ============================================
        # Caso sin salida (ambos lados bloqueados) - FINAL DE RONDA ALEATORIA
        if izquierda > IR_DIR_THRESHOLD and derecha > IR_DIR_THRESHOLD:
            await robot.set_wheel_speeds(0, 0)
            await robot.set_lights_on_rgb(0, 255, 0)  # Verde para finalización
            await robot.play_note(523, 0.5)
            
            # Distancia total recorrida desde inicio
            pos_actual = await robot.get_position()
            dx = pos_actual.x - x0
            dy = pos_actual.y - y0
            distancia_total = (dx**2 + dy**2) ** 0.5
            
            print("\n" + "=" * 50)
            print("RESULTADOS FINALES - ETAPA 04")
            print("=" * 50)
            print(f"Posición inicial: Pose({x0:.2f}, {y0:.2f}, {pos_inicial.heading:.1f}°)")
            print(f"Posición final:   Pose({pos_actual.x:.2f}, {pos_actual.y:.2f}, {pos_actual.heading:.1f}°)")
            print(f"Distancia recorrida: {distancia_total:.2f} cm")
            print("=" * 50)
            print("✓ FINAL DE RONDA ALEATORIA ENCONTRADO")
            print("✓ AMBOS LADOS BLOQUEADOS - SIN SALIDA")
            print("✓ ETAPA 04 COMPLETADA")
            print("=" * 50)
            break  # salir del bucle y terminar

        # ============================================
        # DECISIÓN Y GIRO
        # ============================================
        await robot.set_lights_on_rgb(0, 0, 255)      # AZUL durante giro
        
        if izquierda < derecha:  # Lado izquierdo más libre
            await robot.turn_left(90)
            print("✓ Giro 90° IZQUIERDA (lado más libre)")
        else:  # Lado derecho más libre
            await robot.turn_right(90)
            print("✓ Giro 90° DERECHA (lado más libre)")

        # ============================================
        # AVANCE HASTA OBSTÁCULO
        # ============================================
        print("→ Avanzando hasta siguiente obstáculo...")
        await robot.set_lights_on_rgb(0, 0, 255)      # AZUL durante navegación
        await robot.set_wheel_speeds(5, 5)

        # Esperar hasta obstáculo
        if await detectar_obstaculo(robot):
            await robot.set_wheel_speeds(0, 0)
            await robot.wait(0.2)
            print("✓ Obstáculo detectado - Robot detenido")

            # Señal ROJA + sonido
            await robot.set_lights_on_rgb(255, 0, 0)  # RGB: Rojo
            await robot.play_note(440, 0.5)
            print("✓ Señal de detección: Luz ROJA + sonido")

            # Señal VERDE + sonido
            await robot.set_lights_on_rgb(0, 255, 0)  # RGB: Verde
            await robot.play_note(523, 0.5)
            
            # Distancia total recorrida desde inicio
            pos_actual = await robot.get_position()
            dx = pos_actual.x - x0
            dy = pos_actual.y - y0
            distancia_total = (dx**2 + dy**2) ** 0.5
            print(f"✓ Tramo completado. Distancia acumulada: {distancia_total:.2f} cm")
            print("→ Reanudando inspección para siguiente tramo...")
            continue  # continuar con el siguiente ciclo del bucle


# ==============================================
# Lanzar ejecución
# ==============================================
if __name__ == "__main__":
    robot.play()
