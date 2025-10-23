from irobot_edu_sdk.robots import event
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import Create3

robot = Create3(Bluetooth("C3_UIEC_Grupo1"))

@event(robot.when_play)
async def play(robot):
    # Ejemplo simple: mover 20 cm y terminar el programa.
    await robot.move(20)
    print("Terminando programa...")
    exit()  # Esto finaliza el script y cierra la conexión
