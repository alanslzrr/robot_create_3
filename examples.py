from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Create3
import math

robot = Create3(Bluetooth("C3_UIEC_Grupo1"))

# -------------------------------
# funciones auxiliares
# -------------------------------

async def avanzar(robot, distancia):
    """Avanza la distancia indicada en cm con luces verdes"""
    await robot.set_lights_on_rgb(0, 255, 0)
    await robot.move(distancia)

async def evitar_obstaculo(robot):
    """Luces naranjas, retrocede y gira"""
    await robot.set_lights_on_rgb(255, 80, 0)
    await robot.move(-10)          # retrocede 10 cm
    await robot.turn_left(45)      # gira 45 grados

def hay_obstaculo(sensores, umbral=150):
    """Detecta objeto cercano con el IR central"""
    return sensores[3] > umbral

# -------------------------------
# evento principal
# -------------------------------

@event(robot.when_play)
async def play(robot):
    print("Recorrido recto de 120 cm con evasión de obstáculos")

    restante = 120   # distancia total a recorrer en cm

    while restante > 0:
        sensores = (await robot.get_ir_proximity()).sensors
        if hay_obstaculo(sensores):
            print("Obstáculo detectado, aplicando evasión")
            await evitar_obstaculo(robot)
        else:
            # Avanza tramos cortos para poder verificar continuamente
            paso = min(10, restante)   # máximo 10 cm por paso
            await avanzar(robot, paso)
            restante -= paso
            print(f"Distancia restante: {restante} cm")

        await hand_over()

robot.play()
