#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2022 iRobot Corporation. All rights reserved.
#

# Ejemplo: secuencia de desanclaje y acoplamiento (dock/undock) y visualización del sensor IR del muelle.
# - Cambia `POLL_SENSOR` para comparar sondeo activo vs. eventos.
# - Muestra en LEDs el estado de bits del sensor IR (mapa simple a RGB).

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth("C3_UIEC_Grupo1"))

POLL_SENSOR = True # Cambia para comparar la velocidad de eventos vs sondeo

@event(robot.when_play)
async def play(robot):
    # Dispara un desanclaje y luego anclaje. Puedes repetirlo en bucle.
    print('Undock')
    print(await robot.undock())
    # print('get_docking_values:', await robot.get_docking_values())  # Depuración
    print('Dock')
    print(await robot.dock())
    # print('get_docking_values:', await robot.get_docking_values())  # Depuración

@event(robot.when_play)
async def play(robot):

    while True:
        if POLL_SENSOR:
            sensor = (await robot.get_docking_values())['IR sensor 0']  # Lectura por sondeo
        else:
            sensor = robot.docking_sensor.sensors[0]  # Último valor recibido por evento
            if sensor == None: # aún no se recibió evento
                sensor = 0
        r = 255 * ((sensor & 8)/8)
        g = 255 * ((sensor & 4)/4)
        b = 255 * (sensor & 1)
        await robot.set_lights_on_rgb(r, g, b)

robot.play()
