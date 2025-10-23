#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2021-2022 iRobot Corporation. All rights reserved.
#

# Ejemplo: navegación por puntos formando un cuadrado y su recorrido inverso.
# - Cambia color de LEDs para indicar fases.
# - `navigate_to(x, y)` se usa con coordenadas relativas en cm respecto al origen (0,0).
# - Demuestra movimiento en sentido horario y luego antihorario.

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

# La navegación es específica para cada robot, aquí creamos instancia de Create3.
robot = Create3(Bluetooth())


@event(robot.when_play)
async def play(robot):
    await robot.set_lights_on_rgb(30, 255, 100)
    await robot.play_note(Note.A5, .5)

    distance = 16

    # Recorrer un cuadrado en el plano (0,0) → (0,d) → (d,d) → (d,0) → (0,0)
    await robot.navigate_to(0, distance)
    await robot.navigate_to(distance, distance)
    await robot.navigate_to(distance, 0)
    await robot.navigate_to(0, 0)

    await robot.set_lights_on_rgb(30, 100, 255)

    distance = -distance

    # Recorrido inverso del cuadrado (misma magnitud, signo opuesto)
    await robot.navigate_to(0, distance)
    await robot.navigate_to(distance, distance)
    await robot.navigate_to(distance, 0)
    await robot.navigate_to(0, 0)

    await robot.set_lights_on_rgb(30, 255, 100)

robot.play()
