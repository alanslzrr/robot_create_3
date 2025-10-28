"""
Potenciales para Create 3 – sólo campo ATRACTIVO operativo.
Se deja la firma del repulsivo para la Parte 3.2.
"""
import math

WHEEL_BASE_CM = 23.5          # distancia entre ruedas
V_MAX_CM_S     = 20           # límite seguro de velocidad

def _wrap_pi(a):
    """Normaliza ángulo a (-π, π]."""
    while a >  math.pi: a -= 2*math.pi
    while a <=-math.pi: a += 2*math.pi
    return a

def attractive_wheel_speeds(q, q_goal,
                            k_lin=0.5, k_ang=3.0):
    """
    Devuelve (v_izq, v_der, dist) en cm/s.
    q = (x, y, θ_deg) – posición actual
    q_goal = (xg, yg)   – meta
    No realiza giro-en-sitio: mantiene v>0 salvo en la proximidad.
    """
    dx, dy   = q_goal[0]-q[0], q_goal[1]-q[1]
    dist     = math.hypot(dx, dy)
    theta    = math.radians(q[2])
    desired  = math.atan2(dy, dx)
    ang_err  = _wrap_pi(desired - theta)

    v = min(V_MAX_CM_S, k_lin*dist)          # traslación
    w = k_ang * ang_err                      # rotación (rad/s)

    # convertimos a velocidades de rueda (cm/s)
    v_l = v - (WHEEL_BASE_CM/2.0)*w
    v_r = v + (WHEEL_BASE_CM/2.0)*w
    return v_l, v_r, dist

# *** Hueco para la parte de potencial repulsivo ***
def repulsive_force(*args, **kwargs):
    """ Implementar en la Parte 3.2 """
    return 0.0, 0.0
