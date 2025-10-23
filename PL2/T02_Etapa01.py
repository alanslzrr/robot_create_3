"""
================================================================================
PRÁCTICA DE LABORATORIO 02 - ROBOTS AUTÓNOMOS
ETAPA 01: DETECCIÓN Y PARADA
================================================================================

INFORMACIÓN BÁSICA:
- Autores: Yago Ramos - Salazar Alan
- Fecha de finalización: 7 de octubre de 2025
- Institución: UIE

OBJETIVO:
- Mover el robot en línea recta y detenerse a exactamente 15 cm de un obstáculo
- Utilizar sensores IR frontales para detectar proximidad
- Implementar señalización luminosa y sonora según especificaciones
- Calcular y reportar distancia total recorrida

COMPORTAMIENTO ESPERADO:
1. Reset de navegación y señal inicial (AZUL + sonido)
2. Mostrar posición inicial [x, y, θ] en consola
3. Avanzar a velocidad constante de 5 cm/s
4. Detenerse automáticamente al detectar obstáculo a ~15 cm
5. Señalizar detección (ROJO + sonido) y finalización (VERDE + sonido)
6. Reportar posición final y distancia total recorrida

SENSORES UTILIZADOS:
- IR Proximity Sensor (índice 3): Sensor frontal central para detección de obstáculos
- Umbral IR_OBS_THRESHOLD = 120: Corresponde a aproximadamente 15 cm de distancia
"""

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# =========================
# CONFIGURACIÓN Y PARÁMETROS
# =========================
# Conexión Bluetooth con el robot Create3 del laboratorio
robot = Create3(Bluetooth("C3_UIEC_Grupo1"))

# Umbral de detección de obstáculos
# Valor 120 en el sensor IR frontal corresponde a aproximadamente 15 cm de distancia
# Este valor NO debe cambiarse para mantener la precisión requerida
IR_OBS_THRESHOLD = 120   # ~15 cm (CRÍTICO: no modificar)

# =========================
# FUNCIONES AUXILIARES
# =========================
async def detectar_obstaculo(robot, umbral: int = IR_OBS_THRESHOLD) -> bool:
    """
    FUNCIÓN DE DETECCIÓN DE OBSTÁCULOS
    
    Esta función monitorea continuamente el sensor IR frontal hasta detectar
    un obstáculo a la distancia especificada por el umbral.
    
    Parámetros:
        robot: Instancia del robot Create3
        umbral: Valor umbral para detección (default: IR_OBS_THRESHOLD)
    
    Retorna:
        bool: True cuando se detecta obstáculo y debe detenerse
    
    Lógica:
        - Lee continuamente el array de sensores IR
        - Usa específicamente el sensor frontal central (índice 3)
        - Compara el valor del sensor contra el umbral establecido
        - Retorna True cuando el valor supera el umbral (obstáculo detectado)
    """
    while True:
        # Obtener lectura actual de todos los sensores IR
        ir = (await robot.get_ir_proximity()).sensors
        
        # Verificar que el sensor frontal central esté disponible y supere umbral
        if len(ir) > 3 and ir[3] > umbral:
            return True  # Obstáculo detectado a ~15 cm

# =========================
# EJECUCIÓN PRINCIPAL - ETAPA 01
# =========================
@event(robot.when_play)
async def play(robot):
    """
    SECUENCIA PRINCIPAL DE LA ETAPA 01
    
    Esta función ejecuta la secuencia completa de la Etapa 01 siguiendo
    exactamente las especificaciones del laboratorio.
    
    FLUJO DE EJECUCIÓN:
    1. Reset de odometría y señalización inicial (AZUL + sonido)
    2. Captura de posición inicial para cálculo de distancia
    3. Avance a velocidad constante hasta detección de obstáculo
    4. Detención automática y señalización de estados
    5. Cálculo y reporte de distancia total recorrida
    """
    print("=" * 50)
    print("ETAPA 01: DETECCIÓN Y PARADA A 15 CM")
    print("=" * 50)

    # ============================================
    # PASO A: RESET DE NAVEGACIÓN
    # ============================================
    # Reinicia la odometría del robot para comenzar desde posición conocida
    await robot.reset_navigation()
    print("✓ Reset de navegación completado")

    # ============================================
    # PASO B: SEÑAL INICIAL (AZUL + SONIDO)
    # ============================================
    # Activa luz azul y emite sonido para indicar inicio del movimiento
    await robot.set_lights_on_rgb(0, 0, 255)      # RGB: Azul
    await robot.play_note(440, 0.5)               # Nota A4 por 0.5 segundos
    print("✓ Señal inicial: Luz AZUL + sonido activados")

    # ============================================
    # PASO C: CAPTURA DE POSICIÓN INICIAL
    # ============================================
    # Obtiene la posición inicial [x, y, θ] para posterior cálculo de distancia
    pos0 = await robot.get_position()
    x0, y0, h0 = pos0.x, pos0.y, pos0.heading
    print(f"✓ Posición inicial capturada: Pose({x0:.2f}, {y0:.2f}, {h0:.1f}°)")

    # ============================================
    # PASO D: CONFIGURACIÓN DE VELOCIDAD
    # ============================================
    # Establece velocidad constante de 5 cm/s en ambas ruedas
    await robot.set_wheel_speeds(5, 5)
    print("✓ Velocidad establecida: 5 cm/s")

    # ============================================
    # PASO E: AVANCE Y DETECCIÓN DE OBSTÁCULO
    # ============================================
    print("→ Iniciando avance... monitoreando sensores IR...")
    
    # La función detectar_obstaculo() bloquea hasta detectar obstáculo a ~15 cm
    if await detectar_obstaculo(robot):
        # Detiene inmediatamente las ruedas al detectar obstáculo
        await robot.set_wheel_speeds(0, 0)
        print("✓ Obstáculo detectado - Robot detenido")

    # ============================================
    # PASO F: SEÑAL ROJA (OBSTÁCULO DETECTADO)
    # ============================================
    # Activa luz roja y sonido para indicar detección de obstáculo
    await robot.set_lights_on_rgb(255, 0, 0)      # RGB: Rojo
    await robot.play_note(440, 0.5)               # Nota A4 por 0.5 segundos
    print("✓ Señal de detección: Luz ROJA + sonido")

    # ============================================
    # PASO G: SEÑAL VERDE (FIN DE ETAPA)
    # ============================================
    # Activa luz verde y sonido para indicar finalización exitosa
    await robot.set_lights_on_rgb(0, 255, 0)      # RGB: Verde
    await robot.play_note(523, 0.5)               # Nota C5 por 0.5 segundos
    print("✓ Señal de finalización: Luz VERDE + sonido")

    # ============================================
    # PASO I: CÁLCULO Y REPORTE DE DISTANCIA
    # ============================================
    # Obtiene posición final y calcula distancia euclidiana recorrida
    pos1 = await robot.get_position()
    dx, dy = pos1.x - x0, pos1.y - y0
    dist = (dx**2 + dy**2) ** 0.5
    
    print("\n" + "=" * 50)
    print("RESULTADOS FINALES - ETAPA 01")
    print("=" * 50)
    print(f"Posición inicial: Pose({x0:.2f}, {y0:.2f}, {h0:.1f}°)")
    print(f"Posición final:   Pose({pos1.x:.2f}, {pos1.y:.2f}, {pos1.heading:.1f}°)")
    print(f"Distancia recorrida: {dist:.2f} cm")
    print("=" * 50)
    print("✓ ETAPA 01 COMPLETADA EXITOSAMENTE")
    print("=" * 50)

if __name__ == "__main__":
    robot.play()
