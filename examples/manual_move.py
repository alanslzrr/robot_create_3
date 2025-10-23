"""
Control manual + Logger de sensores para iRobot Create 3
--------------------------------------------------------
Este programa permite controlar el robot con el teclado (WASD)
y al mismo tiempo imprime en consola los valores de todos los
sensores disponibles a través del SDK de iRobot.

- W : avanzar
- S : retroceder
- A : girar izquierda
- D : girar derecha
- ESC : salir

El log muestra:
  * Posición (odometría relativa)
  * Acelerómetro 3D
  * Bumpers (colisiones frontales)
  * Botones de usuario
  * Sensores de proximidad IR
  * Sensores de caída (cliff)
  * Nivel de batería
"""

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import Create3
from pynput import keyboard
import queue
import threading
import time
import math
import asyncio

# -------------------------------
# Conexión al robot por Bluetooth
# -------------------------------
robot = Create3(Bluetooth("C3_UIEC_Grupo1"))

# Velocidades base (cm/s)
VEL = 20   # Avance/retroceso
GIRO = 10  # Diferencia entre ruedas para girar

# Geometría del robot (distancia entre ruedas en cm)
ANCHO_EJE_CM = 23.5
# Velocidad angular aproximada en grados/segundo
DEG_POR_SEG = (2.0 * GIRO / ANCHO_EJE_CM) * (180.0 / math.pi)

# Estado de teclas y cola de comandos
teclas_activas = {'w': False, 'a': False, 's': False, 'd': False}
inicio_tecla = {'w': None, 'a': None, 's': None, 'd': None}
comando_queue = queue.Queue()
connected_event = threading.Event()


# -------------------------------
# Funciones de control manual
# -------------------------------
def calcular_velocidades():
    """ Calcula las velocidades de cada rueda en función de las teclas presionadas """
    izq, der = 0, 0
    if teclas_activas['w']:
        izq += VEL
        der += VEL
    elif teclas_activas['s']:
        izq -= VEL
        der -= VEL
    if teclas_activas['a']:
        izq -= GIRO
        der += GIRO
    elif teclas_activas['d']:
        izq += GIRO
        der -= GIRO
    return izq, der

def actualizar_movimiento():
    """ Actualiza la cola de comandos con las nuevas velocidades """
    izq, der = calcular_velocidades()
    try:
        while not comando_queue.empty():
            comando_queue.get_nowait()
        comando_queue.put((izq, der))
    except:
        pass

def on_press(key):
    """ Se ejecuta al presionar una tecla """
    try:
        if not connected_event.is_set():
            return
        k = key.char.lower()
        if k in teclas_activas and not teclas_activas[k]:
            teclas_activas[k] = True
            inicio_tecla[k] = time.perf_counter()
            actualizar_movimiento()
    except:
        pass

def on_release(key):
    """ Se ejecuta al soltar una tecla """
    try:
        if not connected_event.is_set():
            return
        k = key.char.lower()
        if k in teclas_activas and teclas_activas[k]:
            fin = time.perf_counter()
            inicio = inicio_tecla.get(k)
            if inicio is not None:
                dt = fin - inicio
                # Estimación de distancia/ángulo recorrido
                if k == 'w':
                    print(f"Avanzó {VEL*dt:.1f} cm")
                elif k == 's':
                    print(f"Retrocedió {VEL*dt:.1f} cm")
                elif k == 'a':
                    print(f"Giró {DEG_POR_SEG*dt:.1f}° a la izquierda")
                elif k == 'd':
                    print(f"Giró {DEG_POR_SEG*dt:.1f}° a la derecha")
            inicio_tecla[k] = None
            teclas_activas[k] = False
            actualizar_movimiento()
    except:
        pass
    if key == keyboard.Key.esc:
        return False

# -------------------------------
# Logger de sensores
# -------------------------------
async def log_sensores(robot):
    """
    Lee todos los sensores del Create 3 y los imprime en consola
    con nombres claros.
    """
    while True:
        try:
            # Pose interna del robot
            pos = await robot.get_position()
            x, y, heading = pos.x, pos.y, pos.heading

            # Acelerómetro
            ax, ay, az = await robot.get_accelerometer()

            # Bumpers
            bump_left, bump_right = await robot.get_bumpers()

            # Botones
            btn1, btn2 = await robot.get_touch_sensors()

            # Proximidad IR
            prox = await robot.get_ir_proximity()
            sensors_values = prox.sensors if hasattr(prox, 'sensors') else prox

            # Sensores de caída (cliff)
            cliff = await robot.get_cliff_sensors()

            # Batería
            mV, percent = await robot.get_battery_level()

            # -------------------
            # Impresión legible
            # -------------------
            print("\n=== LOG SENSORES ===")
            print(f"Posición: x={x:.2f} cm, y={y:.2f} cm, θ={heading:.1f}°")
            print(f"Acelerómetro: X={ax}, Y={ay}, Z={az}")
            print(f"Bumpers: Izq={bump_left}, Der={bump_right}")
            print(f"Botones: 1={btn1}, 2={btn2}")

            if sensors_values:
                print("Proximidad IR:")
                for i, value in enumerate(sensors_values):
                    print(f"  {i}: {value}")

            if cliff:
                print("Sensores de Caída (cliff):")
                for i, val in enumerate(cliff):
                    print(f"  {i}: {val}")

            print(f"Batería: {percent}% ({mV} mV)")
            print("====================\n")

        except Exception as e:
            print(f"Error leyendo sensores: {e}")

        await robot.wait(2)  # refresco cada 2 segundos

# -------------------------------
# Loop principal del robot
# -------------------------------
@robot.when_play
async def when_play(robot):
    """ Bucle principal del robot: ejecuta movimiento y logging """
    if not connected_event.is_set():
        connected_event.set()
        print("Conectado al robot. Listo para recibir comandos.")

    # Ejecutar logger en paralelo
    asyncio.create_task(log_sensores(robot))

    # Bucle continuo de movimiento
    while True:
        try:
            if not comando_queue.empty():
                izq, der = comando_queue.get_nowait()
                await robot.set_wheel_speeds(izq, der)
        except Exception as e:
            print(f"Error movimiento: {e}")
        await robot.wait(0.05)  # 50 ms

# -------------------------------
# Main
# -------------------------------
if __name__ == "__main__":
    print("Control WASD en tiempo real:")
    print("  W = avanzar, S = retroceder, A = izquierda, D = derecha")
    print("  Combinaciones: W+A / W+D = curvas")
    print("  ESC = salir")
    print("Conectando al robot...")

    # Hilo para correr la lógica del robot
    hilo_robot = threading.Thread(target=robot.play, daemon=True)
    hilo_robot.start()

    # Espera conexión
    if not connected_event.wait(timeout=20):
        print("No se pudo conectar en 20s. Verifica Bluetooth y el nombre del robot.")
    else:
        # Inicia escucha de teclado
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        listener.join()
