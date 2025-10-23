# Realizado por: Alan Salazar, Yago Ramos
# 3º Grado en ingería en sistemas inteligentes 
# Práctica de laboratorio evaluada 01 
# Robotas autónomos

# Inspeccion completa
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# Conexión al robot por Bluetooth (ajusta el nombre si es necesario)
robot = Create3(Bluetooth("C3_UIEC_Grupo1"))

# Umbral para detectar obstáculo
IR_THRESHOLD = 150
VELOCIDAD_SUAVE = 15  # cm/s

async def has_obstacle(robot):
    """Detecta si hay obstáculo frontal"""
    sensors = (await robot.get_ir_proximity()).sensors
    # Sensor frontal central es sensors[3]
    return sensors[3] > IR_THRESHOLD if len(sensors) > 3 else False


async def inspect_area(robot):
    """Inspecciona el área en busca de obstáculos"""
    print("Iniciando inspección del área...")
    await robot.set_lights_on_rgb(255, 255, 0)
    await robot.play_note(440, 0.4)

    # Hacer un giro de 360 grados para inspeccionar
    for i in range (12):
        await robot.turn_right(30)
        await robot.wait(0.2)

@event(robot.when_play)
async def play(robot):

            # Paso 1
    # Aviso previo: luz roja y pitido
    print("Aviso: luz roja y pitido antes de salir del dock...")
    await robot.set_lights_on_rgb(255, 0, 0)
    await robot.play_note(440, 0.4)
    # Posición tras salir del dock
    print("Posición actual (x, y, heading):", robot.pose)

    # Arranque suave justo tras undock (30 cm)
    print("Arranque suave: avanzando 30 cm a velocidad suave...")
    await robot.move(-30)


            # Paso 2
    # Fijar este punto como origen de navegación (0,0)
    print("Fijando posición actual como origen (0,0)")
    await robot.reset_navigation()

    # Luz verde y pitido para indicar que ha salido del dock
    await robot.set_lights_on_rgb(0, 255, 0)
    await robot.play_note(440, 0.4)
    await robot.wait(1)
    print("Avanzando 180 cm a velocidad suave...")

    # Dar la vuelta
    await robot.set_lights_rgb(0, 0, 255)
    await robot.turn_right(180)

    # Inspeccionar el área
    await inspect_area(robot)
    print("Área despejada.")

    # Volver a Navegar
    print ("NAVEGANDO..")
    await robot.set_lights_rgb(0, 0, 255)
    await robot.move(180)
    print (" El robot se ha movido 180 cm")
    await robot.turn_left(90)
    await robot.set_lights_rgb(0, 255, 0)
    await robot.play_note(440, 0.4)
    await robot.wait(1)

    # Inspeccionar el área
    await inspect_area(robot)
    print("Área despejada.")

            # Paso 3
    # Volver a encende la luz azul
    await robot.set_lights_on_rgb(0, 0, 255)
    # Avanzar 180 cm (velocidad suave)
    print("Avanzando 180 cm a velocidad suave...")
    await robot.move(180)

    # Dar la vuelta (giro de 180 grados)
    await robot.turn_right(180)

    # Finalizando etapa con luz verde y pitido
    await robot.set_lights_on_rgb(0, 255, 0)
    await robot.play_note(440, 0.4)

    # Inspeccionar el área
    await inspect_area(robot)
    print("Área despejada.")

        #Paso 4
    # Volver al dock por el mismo camino
    print("Volviendo al dock por el mismo camino...")
    await robot.set_lights_on_rgb(0, 0, 255)
    await robot.move(160)
    await robot.turn_right(90)

    # Inspeccionar el área
    await inspect_area(robot)
    print("Área despejada.")

    await robot.set_lights_on_rgb(0, 0, 255)
    await robot.move(180)
# Encender luz verde y pitido
    await robot.set_lights_on_rgb(0, 255, 0)
    await robot.play_note(440, 0.4)
    print("Posición actual (x, y, heading):", robot.pose)


"""      Comentado, no es necesario utilizar .dock. sino simplemente quedar ahi
        # Regresar a la estación (dock) usando sensores
        print("Regresando a la estación (dock) usando sensores...")
        await robot.dock()

        # Finalizar con color verde tras acoplar
        await robot.set_lights_on_rgb(0, 255, 0)

        print("Secuencia completa y acoplado.")"""

if __name__ == "__main__":
    print("Iniciando secuencia desde la base...")
    robot.play()