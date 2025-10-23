#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2021-2022 iRobot Corporation. All rights reserved.
#

# Ejemplo: lectura continua de sensores de proximidad IR y salida por consola.
# - `get_ir_proximity()` devuelve un objeto con la lista `sensors`.
# - El índice 0..4 corresponde a los sensores frontales (izq → der) dependiendo del modelo.

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())


@event(robot.when_play)
async def play(robot):
    await robot.set_lights_on_rgb(0, 255, 0)
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        print(sensors)

robot.play()
