"""
Herramienta de teleoperación para definir puntos de navegación

Autores: Alan Salazar, Yago Ramos
Fecha: 4 de noviembre de 2025
Institución: UIE Universidad Intercontinental de la Empresa
Asignatura: Robots Autónomos - Profesor Eladio Dapena
Robot SDK: irobot-edu-sdk

OBJETIVOS PRINCIPALES:

En este módulo implementamos una herramienta de teleoperación que permite al
operador mover el robot manualmente y marcar posiciones específicas que luego
se utilizan como puntos de inicio y fin para la navegación autónoma. Nuestro
objetivo principal era crear una interfaz intuitiva que facilitara la definición
de trayectorias sin necesidad de conocer las coordenadas exactas del espacio.

Los objetivos específicos que buscamos alcanzar incluyen:

1. Proporcionar control manual del robot mediante teclado para moverse libremente
   por el espacio de trabajo
2. Permitir marcado de puntos mediante botones físicos del robot, proporcionando
   una forma natural e intuitiva de definir posiciones
3. Resetear la odometría al inicio para usar el origen (0,0) como referencia
   absoluta, facilitando la reproducibilidad de experimentos
4. Validar que los puntos marcados estén separados una distancia mínima para
   asegurar que la navegación tenga sentido
5. Generar archivos JSON con formato estandarizado que puedan ser leídos fácilmente
   por los scripts principales de navegación
6. Proporcionar realimentación visual en consola para que el operador sepa qué
   puntos han sido marcados y su estado actual

Comportamiento esperado:
    - Permitir control manual del robot con teclas WASD del teclado
    - Resetear la odometría al inicio para usar origen (0,0) como referencia
    - Capturar posición y orientación actual al presionar botón físico 1 (q_i)
    - Capturar posición objetivo al presionar botón físico 2 (q_f)
    - Validar que ambos puntos estén separados al menos 10 cm
    - Generar archivo points.json con formato estandarizado
    - Mostrar realimentación visual de posiciones marcadas en consola

Controles:
    Teclado:
        W - Avanzar recto
        S - Retroceder
        A - Girar a la izquierda
        D - Girar a la derecha
        ESC - Salir de teleoperación
    
    Botones físicos del robot:
        Botón 1 (•) - Guardar posición actual como q_i
        Botón 2 (••) - Guardar posición actual como q_f

Parámetros:
    - VEL: Velocidad de avance/retroceso (15 cm/s desde config.TELEOP_VEL)
    - GIRO: Velocidad de giro (8 cm/s desde config.TELEOP_GIRO)

Salida:
    Archivo points.json con estructura:
    {
        "q_i": {"x": float, "y": float, "theta": float},
        "q_f": {"x": float, "y": float, "theta": float}
    }
"""
import json
import asyncio
import threading
import queue
import time
import math
from pathlib import Path
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import Create3, event
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))
from src import config

# ---------- Conexión ----------
robot = Create3(Bluetooth(config.BLUETOOTH_NAME))
connected_evt, cmd_q = threading.Event(), queue.Queue()
VEL, GIRO = config.TELEOP_VEL, config.TELEOP_GIRO

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
    await robot.reset_navigation()  # Resetear odometría
    print("\n" + "="*60)
    print("🎮 MODO TELEOPERACIÓN - Marcado de Puntos")
    print("="*60)
    print("Controles:")
    print("  W/S = Avanzar/Retroceder")
    print("  A/D = Girar Izquierda/Derecha")
    print("  ESC = Salir")
    print("\nBotones del Robot:")
    print("  Botón 1 (•)  = Marcar q_i (inicio)")
    print("  Botón 2 (••) = Marcar q_f (final)")
    print("="*60)
    
    last_pos_print = time.time()
    
    while True:
        # Mostrar posición cada 2 segundos
        if time.time() - last_pos_print > 2.0:
            pos = await robot.get_position()
            print(f"\n📍 Posición actual: x={pos.x:.1f}, y={pos.y:.1f}, θ={pos.heading:.1f}°")
            if 'q_i' in points:
                print(f"   ✅ q_i marcado: {points['q_i']}")
            if 'q_f' in points:
                print(f"   ✅ q_f marcado: {points['q_f']}")
            last_pos_print = time.time()
        
        # Verificar si ya tenemos ambos puntos
        if len(buttons_pressed) == 2:
            # Validar que sean diferentes
            dist = math.hypot(
                points['q_f']['x'] - points['q_i']['x'],
                points['q_f']['y'] - points['q_i']['y']
            )
            
            if dist < 10.0:
                print("\n⚠️  Los puntos están muy cerca (< 10 cm). Intenta de nuevo.")
                buttons_pressed.clear()
                points.clear()
            else:
                # Guardar y salir
                output_file = Path("data") / config.POINTS_FILE
                output_file.parent.mkdir(exist_ok=True)
                output_file.write_text(json.dumps(points, indent=4))
                print("\n" + "="*60)
                print("✅ PUNTOS GUARDADOS EXITOSAMENTE")
                print("="*60)
                print(f"Archivo: {output_file.absolute()}")
                print(f"q_i: {points['q_i']}")
                print(f"q_f: {points['q_f']}")
                print(f"Distancia: {dist:.1f} cm")
                print("="*60)
                await robot.set_wheel_speeds(0, 0)
                break
        
        # Ejecutar última orden de velocidad
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