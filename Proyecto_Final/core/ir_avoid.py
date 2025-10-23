# core/ir_avoid.py
# Navegación reactiva IR (Bug2 formal) para iRobot Create3
# - SEEK (GOAL_SEEK): avanza hacia el objetivo corrigiendo rumbo por odometría
# - WALL_FOLLOW: bordea el obstáculo manteniéndolo al lado más libre con IR
# - LEAVE (Bug2): regresa a SEEK al cruzar la M-line, estar más cerca que en el "hit"
#   y con error angular razonable (adaptativo). No depende de "frontal libre".
# - Robustez: filtro IIR e histéresis en IR, detección de atasco por proyección en M-line,
#   "arco de despegue" tras LEAVE, ventana refractaria para evitar reenganche inmediato.
#
# Cambios clave:
# - Se añade limitación de delta para mantener SIEMPRE ambas ruedas hacia delante (>= V_MIN)
#   en SEEK, WALL_FOLLOW, _arc_cmd y _leave_arc (evita pivotes sobre el eje).

import math
import time
import asyncio
from typing import Dict, Optional, Sequence, Tuple, List


def _norm_deg(a: float) -> float:
    while a >= 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def _hypot(dx: float, dy: float) -> float:
    return math.hypot(dx, dy)


class IRAvoidNavigator:
    def __init__(self, robot, cfg: Dict, safety=None, telemetry=None) -> None:
        self.robot = robot
        self.safety = safety
        self.telemetry = telemetry

        # -----------------------------
        # Configuración (config.yaml)
        # -----------------------------
        av = cfg.get("avoidance", {})
        mv = cfg.get("motion", {})

        # Umbrales IR (valores validados en laboratorio)
        self.IR_OBS_THRESHOLD = int(av.get("ir_obs_threshold", 120))   # ~15 cm (frontal)
        self.IR_DIR_THRESHOLD = int(av.get("ir_dir_threshold", 200))   # decisión lateral
        # Parámetros de escaneo/arco para rodeo sin detenerse
        self.SCAN_CLEAR = int(av.get("scan_clear_threshold", 140))
        self.ARC_BASE = float(av.get("arc_base_cm_s", 6.0))
        self.ARC_GAIN = float(av.get("arc_gain", 0.006))

        # Índices IR
        self.front_idx: Sequence[int] = av.get("front_idx", [3])
        self.left_idx: Sequence[int] = av.get("left_idx", [0, 1, 2])
        self.right_idx: Sequence[int] = av.get("right_idx", [4, 5, 6])

        # Velocidades (cm/s)
        self.V_FWD = float(av.get("cruise_cm_s", mv.get("vel_default_cm_s", 10.0)))
        self.V_TURN = float(av.get("turn_cm_s", mv.get("giro_default_cm_s", 10.0)))
        self.V_MIN = 2.0

        # Control
        self.KP_GOAL = float(av.get("goal_kp", 0.03))       # deg -> delta cm/s
        self.KP_WALL = float(av.get("wall_kp", 0.004))      # IR  -> delta cm/s
        self.GOAL_TOL_CM = float(av.get("goal_tolerance_cm", 5.0))
        self.REACQUIRE_DEG = float(av.get("reacquire_deg", 12.0))
        self.TIMEOUT_S = float(av.get("timeout_s", 180.0))

        # Stuck detection (por proyección en M-line)
        self.PROGRESS_EPS = float(av.get("progress_eps_cm", 2.0))
        self.PROGRESS_DT = float(av.get("progress_dt_s", 5.0))

        # Límites y temporizaciones
        self.MAX_WHEEL = max(self.V_FWD + self.V_TURN, 30.0)

        # -----------------------------
        # Estado interno de navegación
        # -----------------------------
        self._mline: Optional[Tuple[float, float, float, float]] = None  # (x1,y1,x2,y2)
        self._hit: Optional[Tuple[float, float, float]] = None           # (xh,yh,dist_hit)
        self._wall_side: Optional[str] = None                            # 'left'|'right'
        self._s_hit: Optional[float] = None                              # proyección escalar en hit
        self._last_leave_t: float = -1e9                                 # tiempo del último LEAVE
        self._leave_probe_s: float = 0.35                                # s máx. arco de despegue
        self._leave_refract_s: float = 0.40                              # ventana refractaria LEAVE

        # -----------------------------
        # Filtro IR e histéresis frontal
        # -----------------------------
        self._ir_lp: Optional[List[float]] = None
        self._alpha: float = float(av.get("iir_alpha", 0.20))  # IIR
        # Histéresis: on > obs_th; off < obs_th*0.7
        self._front_on_th = float(self.IR_OBS_THRESHOLD)
        self._front_off_th = float(self.IR_OBS_THRESHOLD) * 0.7
        self._front_blocked_state = False
        self._pivot_start: float | None = None     # límite pivote

    # -----------------------------
    # Utilidades de pose e IR
    # -----------------------------
    async def _pose(self) -> Tuple[float, float, float]:
        p = await self.robot.get_position()
        try:
            return float(p.x), float(p.y), float(p.heading)
        except AttributeError:
            return float(p[0]), float(p[1]), float(p[2])

    async def _ir_raw(self) -> List[float]:
        try:
            prox = await self.robot.get_ir_proximity()
            arr = prox.sensors if hasattr(prox, "sensors") else prox
            if not isinstance(arr, (list, tuple)):
                return []
            # Normalizar a lista de float
            return [float(v) if v is not None else 0.0 for v in arr]
        except Exception:
            return []

    async def _ir_filtered(self) -> List[float]:
        raw = await self._ir_raw()
        if not raw:
            return self._ir_lp if self._ir_lp is not None else []
        if self._ir_lp is None or len(self._ir_lp) != len(raw):
            self._ir_lp = list(raw)
        else:
            a = self._alpha
            self._ir_lp = [a * r + (1.0 - a) * p for r, p in zip(raw, self._ir_lp)]
        return self._ir_lp

    @staticmethod
    def _imax(arr: Sequence[float], idxs: Sequence[int]) -> float:
        vals = [arr[i] for i in idxs if i < len(arr)]
        return max(vals) if vals else 0.0

    # -----------------------------
    # Geometría M-line y métricas
    # -----------------------------
    @staticmethod
    def _proj_s_on_line(x: float, y: float, x1: float, y1: float, x2: float, y2: float) -> float:
        # proyección escalar del punto P(x,y) sobre vector L = (x2-x1, y2-y1)
        vx, vy = x2 - x1, y2 - y1
        lx2 = vx * vx + vy * vy
        if lx2 <= 1e-9:
            return 0.0
        return ((x - x1) * vx + (y - y1) * vy) / math.sqrt(lx2)

    @staticmethod
    def _dist_to_line(x: float, y: float, x1: float, y1: float, x2: float, y2: float) -> float:
        # distancia punto-recta infinita de la M-line
        vx, vy = x2 - x1, y2 - y1
        num = abs(vy * x - vx * y + x2 * y1 - y2 * x1)
        den = math.hypot(vx, vy)
        return num / max(den, 1e-9)

    # -----------------------------
    # Ayudas de control (anti-pivote)
    # -----------------------------
    def _clamp_delta_for_forward(self, base: float, raw_delta: float) -> float:
        """
        Limita |delta| para asegurar que (base ± delta) >= V_MIN.
        Evita que cualquier rueda quede negativa o por debajo de V_MIN.
        """
        max_delta = max(0.0, base - self.V_MIN)
        if raw_delta >  max_delta:
            return  max_delta
        if raw_delta < -max_delta:
            return -max_delta
        return raw_delta

    # -----------------------------
    # Actuación segura de ruedas
    # -----------------------------
    async def _apply(self, vl: float, vr: float) -> None:
        # Saturación uniforme
        mx = max(abs(vl), abs(vr), 1e-9)
        if mx > self.MAX_WHEEL:
            scale = self.MAX_WHEEL / mx
            vl *= scale
            vr *= scale

        # Safety
        if self.safety and getattr(self.safety, "halted", None) and self.safety.halted.is_set():
            await self.robot.set_wheel_speeds(0, 0)
            if self.telemetry:
                self.telemetry.update_command(0.0, 0.0)
            await asyncio.sleep(0.05)
            return

        await self.robot.set_wheel_speeds(vl, vr)
        if self.telemetry:
            self.telemetry.update_command(vl, vr)

    async def _backup_and_turn(self, turn_dir: str = "right", back_s: float = 0.6, turn_deg: float = 30.0) -> None:
        # Pequeña recuperación para salir de esquinas/concavidades
        await self._apply(-self.V_FWD / 2, -self.V_FWD / 2)
        await self.robot.wait(back_s)
        await self._apply(0, 0)
        if turn_dir == "left":
            await self.robot.turn_left(abs(turn_deg))
        else:
            await self.robot.turn_right(abs(turn_deg))

    async def _leave_arc(self, side: str) -> None:
        # Arco corto hacia el lado opuesto a la pared; se interrumpe si cae el frontal
        base = max(self.V_FWD * 0.8, self.V_MIN)
        # Limitar el "giro" para no invertir ruedas
        delta_turn = self._clamp_delta_for_forward(base, self.V_TURN)
        t_end = time.monotonic() + self._leave_probe_s
        while time.monotonic() < t_end:
            if side == "left":   # pared a la izquierda -> curvar a la derecha
                vl, vr = base + delta_turn, base - delta_turn
            else:                # pared a la derecha -> curvar a la izquierda
                vl, vr = base - delta_turn, base + delta_turn
            await self._apply(vl, vr)
            ir = await self._ir_filtered()
            front_val = self._imax(ir, self.front_idx)
            if front_val < self._front_off_th:
                break
            await self.robot.wait(0.04)
        await self._apply(0, 0)

    # -----------------------------
    # Helpers de escaneo y arco
    # -----------------------------
    def _front_trio(self, ir_vals: List[float]) -> Tuple[float, float, float]:
        """
        Devuelve (FL, FC, FR) usando:
          - FL: último índice de left_idx
          - FC: máx de índices frontales
          - FR: primer índice de right_idx
        """
        fl = ir_vals[self.left_idx[-1]] if self.left_idx and self.left_idx[-1] < len(ir_vals) else 0.0
        fc = max(ir_vals[i] for i in self.front_idx if i < len(ir_vals)) if self.front_idx else 0.0
        fr = ir_vals[self.right_idx[0]] if self.right_idx and self.right_idx[0] < len(ir_vals) else 0.0
        return fl, fc, fr

    def _arc_cmd(self, fl: float, fc: float, fr: float) -> Tuple[float, float]:
        base = max(self.ARC_BASE, self.V_MIN)
        diff = (fr - fl)  # >0 => más cerca a la derecha; <0 => más cerca a la izquierda
        raw_delta = self.ARC_GAIN * diff
        delta = self._clamp_delta_for_forward(base, raw_delta)
        vl = base + delta
        vr = base - delta
        return vl, vr

    def _front_center_value(self, ir_vals: List[float]) -> float:
        """Valor del IR frontal central. Si índice 3 existe en front_idx, usarlo; si no, usar el índice de front_idx más cercano a 3."""
        if not self.front_idx:
            return 0.0
        if 3 in self.front_idx and 3 < len(ir_vals):
            return ir_vals[3]
        best_i = min(self.front_idx, key=lambda i: abs(i - 3))
        return ir_vals[best_i] if best_i < len(ir_vals) else 0.0

    def _front_any_over(self, ir_vals: List[float], threshold: float) -> bool:
        """True si cualquier índice frontal supera el umbral."""
        for i in self.front_idx:
            if i < len(ir_vals) and ir_vals[i] > threshold:
                return True
        return False

    # -----------------------------
    # Bucle principal de navegación
    # -----------------------------
    async def go_to(self, x_goal: float, y_goal: float, time_limit_s: Optional[float] = None):
        """
        Navega hacia (x_goal, y_goal) bordeando obstáculos con IR (Bug2 formal).
        Retorna: (ok: bool, pose_final: (x,y,theta)).
        """
        t0 = time.monotonic()
        limit = float(time_limit_s or self.TIMEOUT_S)

        # Estado inicial
        x0, y0, th0 = await self._pose()
        self._mline = (x0, y0, x_goal, y_goal)
        self._hit = None
        self._s_hit = None
        self._wall_side = None
        self._front_blocked_state = False  # reset histéresis
        s_last = 0.0
        last_prog_t = t0

        STATE = "SEEK"
        await self.robot.set_lights_on_rgb(0, 128, 255)  # azul = navegando

        while True:
            # Timeout
            if (time.monotonic() - t0) > limit:
                await self._apply(0, 0)
                await self.robot.set_lights_on_rgb(255, 165, 0)  # naranja = timeout
                x, y, th = await self._pose()
                return False, (x, y, th)

            # Pose y objetivo
            x, y, th = await self._pose()
            dx, dy = x_goal - x, y_goal - y
            dist = _hypot(dx, dy)
            if dist <= self.GOAL_TOL_CM:
                await self._apply(0, 0)
                await self.robot.set_lights_on_rgb(0, 255, 0)  # verde = éxito
                return True, (x, y, th)

            # Rumbo al objetivo
            goal_heading = math.degrees(math.atan2(dy, dx))
            heading_err = _norm_deg(goal_heading - th)

            # Progreso por proyección en M-line
            x1, y1, x2, y2 = self._mline
            s_now = self._proj_s_on_line(x, y, x1, y1, x2, y2)
            now = time.monotonic()
            # Detección de atasco: solo en SEEK para no interrumpir WALL_FOLLOW
            if STATE == "SEEK" and (now - last_prog_t) > self.PROGRESS_DT:
                if (s_now - s_last) < self.PROGRESS_EPS:
                    # Recuperación por falta de progreso escalar
                    await self._backup_and_turn("right" if heading_err >= 0 else "left", back_s=0.6, turn_deg=30.0)
                s_last = s_now
                last_prog_t = now

            # IR filtrado + histéresis frontal
            ir = await self._ir_filtered()
            front_val = self._imax(ir, self.front_idx)
            left_val = self._imax(ir, self.left_idx)
            right_val = self._imax(ir, self.right_idx)

            # Actualizar frontal_blocked con histéresis
            if self._front_blocked_state:
                # Salir cuando el sensor central cae por debajo del umbral OFF
                center_val = self._front_center_value(ir)
                self._front_blocked_state = center_val > self._front_off_th
            else:
                # Entrar cuando CUALQUIERA de los frontales supera el umbral ON
                self._front_blocked_state = self._front_any_over(ir, self._front_on_th)
            front_blocked = self._front_blocked_state

            if STATE == "SEEK":
                # Corrección de rumbo proporcional con anti-pivote
                base = max(self.V_FWD, self.V_MIN)
                raw_delta = self.KP_GOAL * heading_err
                delta = self._clamp_delta_for_forward(base, raw_delta)
                vl, vr = base - delta, base + delta

                if front_blocked:
                    # Selección robusta del lado libre (margen 10 %)
                    margin = 0.10 * self.IR_DIR_THRESHOLD
                    if abs(left_val - right_val) < margin and self._wall_side:
                        pass  # mantener lado anterior
                    else:
                        self._wall_side = "left" if left_val < right_val else "right"
                    await self._apply(0, 0)
                    STATE = "WALL_FOLLOW"
                    # Memorizar "hit"
                    if self._hit is None:
                        self._hit = (x, y, dist)
                    # Proyección escalar en hit
                    self._s_hit = self._proj_s_on_line(x, y, x1, y1, x2, y2)
                    # Micro-giro hacia el lado libre para enganchar pared
                    if self._wall_side == "left":
                        await self.robot.turn_left(20)
                    else:
                        await self.robot.turn_right(20)
                    continue
                else:
                    await self._apply(vl, vr)

            elif STATE == "WALL_FOLLOW":
                # Bordeo: mantener obstáculo al lado elegido con control lateral, avanzando siempre
                base = max(self.V_FWD * 0.7, self.V_MIN)
                if self._wall_side == "left":
                    err_lat = left_val - (0.8 * self.IR_DIR_THRESHOLD)
                    raw_delta = self.KP_WALL * err_lat
                    delta = self._clamp_delta_for_forward(base, raw_delta)
                    vl, vr = base + delta, base - delta  # alejamiento pared
                else:
                    err_lat = right_val - (0.8 * self.IR_DIR_THRESHOLD)
                    raw_delta = self.KP_WALL * err_lat
                    delta = self._clamp_delta_for_forward(base, raw_delta)
                    vl, vr = base - delta, base + delta

                if front_blocked:
                    # Escaneo con FL/FC/FR + avance en arco para rodear sin detenerse
                    t_scan = time.monotonic()
                    while True:
                        ir = await self._ir_filtered()
                        fl, fc, fr = self._front_trio(ir)
                        # salida: frontal CENTRAL suficientemente despejado (histéresis OFF)
                        center_val = self._front_center_value(ir)
                        if center_val < self._front_off_th:
                            break
                        left_clear  = (fl < self.SCAN_CLEAR)
                        right_clear = (fr < self.SCAN_CLEAR)
                        avl, avr = self._arc_cmd(fl, fc, fr)
                        # fuerza el sentido si un lado está claro y el otro no
                        if left_clear and not right_clear and avl > avr:
                            avl, avr = avr, avl    # asegurar giro a la IZQUIERDA
                        elif right_clear and not left_clear and avl < avr:
                            avl, avr = avr, avl    # asegurar giro a la DERECHA
                        await self._apply(avl, avr)
                        await self.robot.wait(0.06)
                        # corta por seguridad si no encuentra hueco
                        if time.monotonic() - t_scan > 3.0:
                            break
                    self._pivot_start = None
                else:
                    self._pivot_start = None

                await self._apply(vl, vr)

                # LEAVE (Bug2): condición geométrica + mejora de distancia + avance más allá del hit.
                d_to_line = self._dist_to_line(x, y, x1, y1, x2, y2)
                closer_than_hit = (self._hit is None) or (dist < self._hit[2] - 2.0)
                progressed_past_hit = (self._s_hit is None) or (s_now > self._s_hit + 1.0)

                # Reacquire adaptativo: permite mayor error angular si estás muy cerca del goal (hasta 30°)
                theta_allow = max(self.REACQUIRE_DEG, min(30.0, 0.2 * dist))
                refract_ok = (now - self._last_leave_t) > self._leave_refract_s

                if refract_ok and (d_to_line < 3.0) and closer_than_hit and progressed_past_hit and (abs(heading_err) <= theta_allow):
                    # Arco de despegue para reducir frontal y evitar reenganche inmediato
                    await self._leave_arc(self._wall_side)
                    STATE = "SEEK"
                    self._last_leave_t = time.monotonic()
                    await self.robot.wait(0.05)
                    continue

            # Paso de control
            await self.robot.wait(0.06)
