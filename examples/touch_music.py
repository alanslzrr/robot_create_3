#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2021-2023 iRobot Corporation. All rights reserved.
#

# Ejemplo: reproducir notas y cambiar luces cuando se tocan los botones táctiles.
# - Usa eventos de toque `when_touched` para detectar el estado de los dos sensores táctiles.
# - Enciende los LEDs con un color específico y reproduce una nota distinta para cada combinación.
# - Al iniciar (`when_play`) reproduce una nota corta para indicar que el programa comenzó.

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())

duration = 0.15


# Evento: se toca el botón izquierdo (.)
@event(robot.when_touched, [True, False])  # Botón izquierdo
async def touched(robot):
    await robot.set_lights_on_rgb(255, 0, 0)
    await robot.play_note(Note.A4, duration)


# Evento: se toca el botón derecho (..)
@event(robot.when_touched, [False, True])  # Botón derecho
async def touched(robot):
    await robot.set_lights_on_rgb(0, 255, 0)
    await robot.play_note(Note.C5_SHARP, duration)


# Evento: se tocan ambos botones a la vez
@event(robot.when_touched, [True, True])
async def touched(robot):
    print('ANY sensor touched')


# Evento de inicio del programa
@event(robot.when_play)
async def play(robot):
    await robot.play_note(Note.A5, duration)

robot.play()
