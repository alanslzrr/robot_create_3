"""
Parte 3.1 – Navegación con potencial atractivo.
· Lee q_i y q_f de points.json.
· Ejecuta bucle de control (20 Hz) sin 'girar-luego-avanzar'.
· Muestra sensores en tiempo real (1 s) vía teleop_logger.
"""
import json, threading
from pathlib import Path
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import Create3, event
from teleop_logger import start_logger
from potential_fields import attractive_wheel_speeds

TOL_DIST_CM = 2                      # criterio de llegada

# ---------- Cargar puntos ----------
if not Path('points.json').exists():
    raise FileNotFoundError("Falta points.json – ejecuta primero point_manager.py")
data = json.loads(Path('points.json').read_text())
Q_I = (data['q_i']['x'], data['q_i']['y'], data['q_i']['theta'])
Q_F = (data['q_f']['x'], data['q_f']['y'])      # sin orientación

# ---------- Conectar robot ----------
robot = Create3(Bluetooth("C3_UIEC_Grupo1"))

@event(robot.when_play)
async def play(robot):
    await robot.reset_navigation()             # origen en q_i
    start_logger(robot)                        # ==> impresión cada 1 s
    print("🔄 Iniciando navegación…")
    while True:
        pos   = await robot.get_position()
        q     = (pos.x, pos.y, pos.heading)
        v_l, v_r, d = attractive_wheel_speeds(q, Q_F)
        await robot.set_wheel_speeds(v_l, v_r)
        if d < TOL_DIST_CM:
            print("🎯 Meta alcanzada")
            await robot.set_wheel_speeds(0,0)
            break
        await robot.wait(0.05)                 # ≈20 Hz bucle de control

if __name__=="__main__":
    threading.Thread(target=robot.play, daemon=True).start()
