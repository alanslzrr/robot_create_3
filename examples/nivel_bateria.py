from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# Conexión al robot por Bluetooth (ajusta el nombre si es necesario)
robot = Create3(Bluetooth("C3_UIEC_Grupo1"))

@event(robot.when_play)
async def play(robot):

    print("Nivel de batería:", await robot.get_battery_level(), "%")

if __name__ == "__main__":
    print("Iniciando secuencia desde la base...")
    robot.play()