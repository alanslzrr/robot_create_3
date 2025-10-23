
#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2021-2023 iRobot Corporation. All rights reserved.
#

# Programa de prueba: navegación y lectura de pose con un movimiento.
# - `print_pos` imprime la pose actual `(x, y, heading)`.
# - Se prueba `navigate_to`, `reset_navigation`, `turn_left` y `move`.

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth("C3_UIEC_Grupo1"))

def print_pos(robot):
    print('🐢 (x  y  heading) =', robot.pose)  # Pose estimada por el sistema de navegación


@event(robot.when_play)
async def play(robot):
    await robot.set_lights_on_rgb(30, 255, 100)
    await robot.play_note(Note.A5, .5)

    distance = 5

    await robot.navigate_to(distance, distance)
    print_pos(robot)
    await robot.reset_navigation()
    print_pos(robot)
    await robot.navigate_to(distance, distance)
    print_pos(robot)

    await robot.turn_left(90)
    print_pos(robot)
    await robot.move(-distance)
    print_pos(robot)

    await robot.set_lights_on_rgb(30, 255, 100)

robot.play()
