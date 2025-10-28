"""
Permite tele-operar al robot (WASD) y guardar q_i y q_f
al pulsar los botones físicos del Create 3:
 - Botón 1 (•)  → posición inicial q_i
 - Botón 2 (••) → posición final   q_f
Tras marcar los dos puntos se crea/actualiza points.json
"""
import json, asyncio, threading, queue, time, math
from pathlib import Path
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import Create3, event

# ---------- Conexión ----------
robot = Create3(Bluetooth("C3_UIEC_Grupo1"))
connected_evt, cmd_q = threading.Event(), queue.Queue()
VEL, GIRO = 20, 10           # cm/s

# ---------- Tele-operación (teclado) ----------
try:
    from pynput import keyboard            # ⬅ mismo esquema que tu ejemplo
except ImportError:
    keyboard = None                        # (por si usas Web Playground)

teclas = {'w':False,'a':False,'s':False,'d':False}
def _calc_speeds():
    v_l=v_r=0
    if teclas['w']: v_l+=VEL; v_r+=VEL
    if teclas['s']: v_l-=VEL; v_r-=VEL
    if teclas['a']: v_l-=GIRO; v_r+=GIRO
    if teclas['d']: v_l+=GIRO; v_r-=GIRO
    return v_l, v_r

def _update_cmd():
    while not cmd_q.empty(): cmd_q.get_nowait()
    cmd_q.put(_calc_speeds())

if keyboard:
    def _on_press(k):
        if not connected_evt.is_set(): return
        if hasattr(k,'char') and k.char and k.char.lower() in teclas:
            teclas[k.char.lower()]=True; _update_cmd()
    def _on_release(k):
        if not connected_evt.is_set(): return
        if hasattr(k,'char') and k.char and k.char.lower() in teclas:
            teclas[k.char.lower()]=False; _update_cmd()
        if k==keyboard.Key.esc: return False   # sale

# ---------- Eventos de los botones del robot ----------
points, buttons_pressed = {}, set()

@event(robot.when_touched, [True, False])       # Botón 1 (izq)
async def btn1(robot):
    global points
    pos = await robot.get_position()
    points['q_i'] = {'x':pos.x, 'y':pos.y, 'theta':pos.heading}
    buttons_pressed.add('q_i')
    print(f"q_i guardado: {points['q_i']}")

@event(robot.when_touched, [False, True])       # Botón 2 (der)
async def btn2(robot):
    global points
    pos = await robot.get_position()
    points['q_f'] = {'x':pos.x, 'y':pos.y, 'theta':pos.heading}
    buttons_pressed.add('q_f')
    print(f"q_f guardado: {points['q_f']}")

# ---------- Bucle principal ----------
@event(robot.when_play)
async def play(robot):
    connected_evt.set()
    print("MANUAL MODE: mueve con WASD y pulsa • para q_i, •• para q_f.")
    while True:
        if len(buttons_pressed)==2:
            # Guardamos JSON y salimos
            Path('points.json').write_text(json.dumps(points, indent=4))
            print("✅ points.json creado. ¡Listo!")
            await robot.set_wheel_speeds(0,0)
            break
        # ejecuta última orden de velocidad
        if not cmd_q.empty():
            v_l, v_r = cmd_q.get_nowait()
            await robot.set_wheel_speeds(v_l, v_r)
        await robot.wait(0.05)

def main():
    th = threading.Thread(target=robot.play, daemon=True)
    th.start()
    if keyboard:
        with keyboard.Listener(on_press=_on_press,on_release=_on_release) as l:
            l.join()

if __name__=="__main__": main()
