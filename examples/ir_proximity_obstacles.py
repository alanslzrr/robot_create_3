#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2021-2022 iRobot Corporation. All rights reserved.
#

# Ejemplo muy básico para evitar obstáculos frontales con sensores IR.
# - Avanza con LEDs en verde.
# - Si el sensor frontal central supera el umbral `th`, retrocede y gira a la izquierda.
# - Repite el ciclo continuamente.

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 10
th = 150


async def forward(robot):
    # Estado normal: avanzar recto con indicador verde
    await robot.set_lights_on_rgb(0, 255, 0)
    await robot.set_wheel_speeds(speed, speed)


async def backoff(robot):
    # Maniobra de evasión: retroceder y girar
    await robot.set_lights_on_rgb(255, 80, 0)
    await robot.move(-20)
    await robot.turn_left(45)


def front_obstacle(sensors):
    # El sensor central frontal suele ser `sensors[3]`
    print(sensors[3])
    return sensors[3] > th


@event(robot.when_play)
async def play(robot):
    await forward(robot)
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        if front_obstacle(sensors):
            await backoff(robot)
            await forward(robot)

robot.play()
