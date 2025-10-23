# Paso 3
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


@event(robot.when_play)
async def play(robot):

            # Paso 1
    # Aviso previo: luz roja y pitido
    print("Aviso: luz roja y pitido antes de salir del dock...")
    await robot.set_lights_on_rgb(255, 0, 0)
    await robot.play_note(440, 0.4)
    # Posición tras salir del dock
    print("Estación de carga", robot.pose)

    # Arranque suave justo tras undock (20 cm)
    print("Arranque suave: avanzando 20 cm a velocidad suave...")
    await robot.move(-20)

    # Fijar punto Inicio misión
    print("Punto inicio misión", robot.pose)

    # Apagar luz roja tras salir del dock
    print ("Apagando luz roja")
    await robot.set_lights_on_rgb(0, 0, 0)

            # Paso 2
    # Luz verde y pitido para indicar que ha salido del dock
    await robot.set_lights_on_rgb(0, 255, 0)
    await robot.play_note(440, 0.4)
    print("Avanzando 180 cm a velocidad suave...")

    # Luz azul para indicar que estamos navegando
    print ("NAVEGANDO..")
    await robot.set_lights_rgb(0, 0, 255)
    await robot.turn_right(180)
    await robot.move(180)
    print (" El robot se ha movido 180 cm")
    await robot.turn_left(90)
    await robot.set_lights_rgb(0, 255, 0)
    await robot.wait(1)

            # Paso 3
    # Volver a encende la luz azul
    await robot.set_lights_on_rgb(0, 0, 255)
    # Avanzar 180 cm (velocidad suave)
    print("Avanzando 180 cm a velocidad suave...")
    await robot.move (180)

    # Dar la vuelta (giro de 180 grados)
    await robot.turn_right(180)

    # Finalizando etapa con luz verde y pitido
    await robot.set_lights_on_rgb(0, 255, 0)
    await robot.play_note(440, 0.4)

    # apagando luz verde
    await robot.set_lights_on_rgb(0, 0, 0)

if __name__ == "__main__":
    print("Iniciando secuencia desde la base...")
    robot.play()