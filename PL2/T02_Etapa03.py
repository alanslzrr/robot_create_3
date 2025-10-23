"""
================================================================================
PRÁCTICA DE LABORATORIO 02 - ROBOTS AUTÓNOMOS
ETAPA 03: DETECTAR Y CONTINUAR II
================================================================================

INFORMACIÓN BÁSICA:
- Autores: Yago Ramos - Salazar Alan
- Fecha de finalización: 7 de octubre de 2025
- Institución: UIE
- Valoración: 2 Puntos

OBJETIVO:
- Repetir las Etapas 1 y 2 completas
- Aplicar el mismo modelo de decisión múltiples veces
- Determinar dirección a seguir para encontrar la siguiente parada
- Reportar distancia total recorrida desde el inicio

COMPORTAMIENTO ESPERADO:
1. Ejecutar Etapa 01 completa (reset, avance, parada a 15 cm)
2. Ejecutar Etapa 02 completa (inspección, giro, avance hasta obstáculo)
3. Repetir nuevamente el modelo de Etapa 02
4. Utilizar señales luminosas y sonoras según especificaciones
5. Reportar distancia total recorrida al final

SENSORES UTILIZADOS:
- IR Proximity Sensor (índice 3): Detección frontal de obstáculos (~15 cm)
- IR Proximity Sensors (índices 0,1): Sensores laterales izquierdos
- IR Proximity Sensors (índices 5,6): Sensores laterales derechos
- IR_DIR_THRESHOLD = 200: Umbral para decisión de giro

LÓGICA DE REPETICIÓN:
- Etapa 01: Una vez (avance inicial hasta primer obstáculo)
- Etapa 02: Dos veces (dos ciclos de inspección + giro + avance)
- Total: 3 tramos de navegación (1 inicial + 2 adicionales)
"""

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# =========================
# CONFIGURACIÓN Y PARÁMETROS
# =========================
# Conexión Bluetooth con el robot Create3 del laboratorio
robot = Create3(Bluetooth("C3_UIEC_Grupo1"))

# Umbrales de sensores IR
IR_OBS_THRESHOLD = 120   # ~15 cm para detección frontal de obstáculos
IR_DIR_THRESHOLD = 200   # Umbral para decisión de giro (CRÍTICO: no cambiar)

# =========================
# FUNCIONES AUXILIARES
# =========================
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

def lado_mas_libre(ir):
    """
    FUNCIÓN DE ANÁLISIS DE LIBERTAD LATERAL
    
    Analiza los sensores laterales para determinar qué lado tiene más espacio libre.
    Utiliza el valor máximo (peor caso) de cada lado para mayor confiabilidad.
    
    Parámetros:
        ir: Array de valores de sensores IR
    
    Retorna:
        tuple: (izquierda, derecha) - valores máximos de cada lado
               Un valor MENOR indica lado MÁS LIBRE
    
    Lógica de Sensores:
        - Izquierda: sensores 0 y 1 (lado izquierdo del robot)
        - Derecha: sensores 5 y 6 (lado derecho del robot)
        - Se toma el máximo de cada lado para detectar el peor obstáculo
    """
    # Calcular valor máximo para lado izquierdo (sensores 0,1)
    izq = max([ir[i] for i in (0,1) if i < len(ir)], default=0)
    
    # Calcular valor máximo para lado derecho (sensores 5,6)
    der = max([ir[i] for i in (5,6) if i < len(ir)], default=0)
    
    return izq, der

async def tramo_seleccion_y_avance(robot):
    """
    FUNCIÓN DE TRAMO COMPLETO: INSPECCIÓN + GIRO + AVANCE
    
    Realiza un ciclo completo de:
    1. Inspección lateral con luz AMARILLA
    2. Decisión de giro basada en sensores laterales
    3. Giro de 90° hacia el lado más libre
    4. Avance hasta próximo obstáculo
    5. Señalización de obstáculo detectado
    
    Retorna:
        True: Si se completa el tramo exitosamente (obstáculo detectado)
        False: Si ambos lados están bloqueados (sin salida)
    """
    # ============================================
    # INSPECCIÓN LATERAL
    # ============================================
    await robot.set_lights_on_rgb(255, 255, 0)    # AMARILLO durante inspección
    ir = (await robot.get_ir_proximity()).sensors
    izq, der = lado_mas_libre(ir)
    print(f"→ Inspección lateral: Izq={izq}, Der={der}")

    # Verificar si ambos lados están bloqueados
    if izq > IR_DIR_THRESHOLD and der > IR_DIR_THRESHOLD:
        print("SIN SALIDA: Ambos lados bloqueados")
        return False

    # ============================================
    # DECISIÓN Y GIRO
    # ============================================
    await robot.set_lights_on_rgb(0, 0, 255)      # AZUL durante giro
    
    if izq < der:  # Lado izquierdo más libre
        await robot.turn_left(90)
        print("✓ Giro 90° IZQUIERDA")
    else:  # Lado derecho más libre
        await robot.turn_right(90)
        print("✓ Giro 90° DERECHA")

    # ============================================
    # AVANCE HASTA OBSTÁCULO
    # ============================================
    await robot.set_lights_on_rgb(0, 0, 255)      # AZUL durante navegación
    await robot.set_wheel_speeds(5, 5)

    # Detectar obstáculo
    if await detectar_obstaculo(robot):
        await robot.set_wheel_speeds(0, 0)
        
        # Señalización de obstáculo detectado
        await robot.set_lights_on_rgb(255, 0, 0)  # ROJO: obstáculo
        await robot.play_note(440, 0.5)
        
        await robot.set_lights_on_rgb(0, 255, 0)  # VERDE: fin de tramo
        await robot.play_note(523, 0.5)
        
        print("✓ Tramo completado: Obstáculo detectado")
        return True

# =========================
# EJECUCIÓN PRINCIPAL - ETAPA 03
# =========================
@event(robot.when_play)
async def play(robot):
    """
    SECUENCIA PRINCIPAL DE LA ETAPA 03
    
    FLUJO DE EJECUCIÓN:
    1. Ejecutar Etapa 01 completa (reset, avance, parada a 15 cm)
    2. Ejecutar Etapa 02 primera vez (inspección + giro + avance)
    3. Ejecutar Etapa 02 segunda vez (inspección + giro + avance)
    4. Reportar distancia total recorrida desde el inicio
    
    TOTAL DE TRAMOS: 3 (1 inicial + 2 adicionales)
    """
    print("=" * 50)
    print("ETAPA 03: DETECTAR Y CONTINUAR II")
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
    pos0 = await robot.get_position()
    x0, y0 = pos0.x, pos0.y
    print(f"✓ Posición inicial: Pose({pos0.x:.2f}, {pos0.y:.2f}, {pos0.heading:.1f}°)")

    # d) Velocidad 5 cm/s
    await robot.set_wheel_speeds(5, 5)
    print("✓ Velocidad establecida: 5 cm/s")

    # e) Avanzar hasta obstáculo a ~15 cm
    print("→ Avanzando hasta primer obstáculo...")
    if await detectar_obstaculo(robot):
        await robot.set_wheel_speeds(0, 0)
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
    # ETAPA 02 PRIMERA VEZ - TRAMO 1
    # ============================================
    print("\n→ Ejecutando Etapa 02 - Tramo 1...")
    
    ok = await tramo_seleccion_y_avance(robot)
    if not ok:
        print("Fin anticipado: Ambos lados bloqueados en Tramo 1")
        # Reportar posición final y distancia recorrida hasta aquí
        pos1 = await robot.get_position()
        dx, dy = pos1.x - x0, pos1.y - y0
        dist = (dx**2 + dy**2) ** 0.5
        print(f"✓ Posición final: Pose({pos1.x:.2f}, {pos1.y:.2f}, {pos1.heading:.1f}°)")
        print(f"✓ Distancia recorrida: {dist:.2f} cm")
        return

    # ============================================
    # ETAPA 02 SEGUNDA VEZ - TRAMO 2
    # ============================================
    print("\n→ Ejecutando Etapa 02 - Tramo 2...")
    
    ok = await tramo_seleccion_y_avance(robot)
    if not ok:
        print("⚠️  Fin anticipado: Ambos lados bloqueados en Tramo 2")
        # Reportar posición final y distancia recorrida hasta aquí
        pos1 = await robot.get_position()
        dx, dy = pos1.x - x0, pos1.y - y0
        dist = (dx**2 + dy**2) ** 0.5
        print(f"✓ Posición final: Pose({pos1.x:.2f}, {pos1.y:.2f}, {pos1.heading:.1f}°)")
        print(f"✓ Distancia recorrida: {dist:.2f} cm")
        return

    # ============================================
    # FINALIZACIÓN EXITOSA
    # ============================================
    print("\n✓ Etapa 03 completada exitosamente - Todos los tramos ejecutados")
    await robot.set_lights_on_rgb(0, 255, 0)      # Verde final
    await robot.play_note(523, 0.5)
    
    # ============================================
    # REPORTE FINAL DE DISTANCIA TOTAL
    # ============================================
    pos1 = await robot.get_position()
    dx, dy = pos1.x - x0, pos1.y - y0
    dist = (dx**2 + dy**2) ** 0.5
    
    print("\n" + "=" * 50)
    print("RESULTADOS FINALES - ETAPA 03")
    print("=" * 50)
    print(f"Posición inicial: Pose({pos0.x:.2f}, {pos0.y:.2f}, {pos0.heading:.1f}°)")
    print(f"Posición final:   Pose({pos1.x:.2f}, {pos1.y:.2f}, {pos1.heading:.1f}°)")
    print(f"Distancia recorrida: {dist:.2f} cm")
    print("Tramos completados: 3 (Etapa 01 + 2x Etapa 02)")
    print("=" * 50)
    print("✓ ETAPA 03 COMPLETADA EXITOSAMENTE")
    print("=" * 50)

if __name__ == "__main__":
    robot.play()
