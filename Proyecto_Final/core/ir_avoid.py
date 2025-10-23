# core/ir_avoid.py
# Navegación reactiva IR (Bug2 ROBUSTO Y COMPLETO) para iRobot Create3
# Algoritmo Bug2 con evasión reactiva basada en sensores IR.
#
# Estados:
# - SEEK: Avanza directo al objetivo con corrección de rumbo
# - WALL_FOLLOW: Bordea obstáculo manteniendo consciencia del objetivo
# - LEAVE: Condición Bug2 para regresar a SEEK
#
# Características robustas:
# - Selección de lado considerando objetivo + sensores
# - Control híbrido: pared + bias hacia objetivo
# - Velocidad adaptativa por IR
# - Logs detallados para debugging

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

        # Umbrales IR
        self.IR_OBS_THRESHOLD = int(av.get("ir_obs_threshold", 120))
        self.IR_DIR_THRESHOLD = int(av.get("ir_dir_threshold", 200))
        self.SCAN_CLEAR = int(av.get("scan_clear_threshold", 140))
        
        # Velocidad adaptativa
        self.IR_FREE_TH = float(av.get("free_threshold", 50.0))
        self.IR_SLOWDOWN_TH = float(av.get("slowdown_threshold", 100.0))
        self.IR_CRITICAL_TH = float(av.get("critical_threshold", 1000.0))
        self.SLOW_FACTOR_MED = float(av.get("slow_factor_med", 0.7))
        self.SLOW_FACTOR_SLOW = float(av.get("slow_factor_slow", 0.35))
        self.SLOW_MIN = float(av.get("slow_min_cm_s", 3.0))

        # Índices IR
        self.front_idx: Sequence[int] = av.get("front_idx", [3])
        self.left_idx: Sequence[int] = av.get("left_idx", [0, 1, 2])
        self.right_idx: Sequence[int] = av.get("right_idx", [4, 5, 6])

        # Velocidades (cm/s)
        self.V_FWD = float(av.get("cruise_cm_s", mv.get("vel_default_cm_s", 10.0)))
        self.V_TURN = float(av.get("turn_cm_s", mv.get("giro_default_cm_s", 10.0)))
        self.V_MIN = 2.0

        # Control
        self.KP_GOAL = float(av.get("goal_kp", 0.03))
        self.KP_WALL = float(av.get("wall_kp", 0.004))
        self.GOAL_TOL_CM = float(av.get("goal_tolerance_cm", 5.0))
        self.REACQUIRE_DEG = float(av.get("reacquire_deg", 15.0))
        self.TIMEOUT_S = float(av.get("timeout_s", 180.0))

        # Stuck detection
        self.PROGRESS_EPS = float(av.get("progress_eps_cm", 2.0))
        self.PROGRESS_DT = float(av.get("progress_dt_s", 5.0))

        # Límites
        self.MAX_WHEEL = max(self.V_FWD + self.V_TURN, 30.0)

        # Estado interno de navegación
        self._mline: Optional[Tuple[float, float, float, float]] = None
        self._hit: Optional[Tuple[float, float, float]] = None
        self._wall_side: Optional[str] = None
        self._s_hit: Optional[float] = None
        self._last_leave_t: float = -1e9
        self._leave_refract_s: float = 0.50
        self._corner_turn_count: int = 0  # Contador de giros consecutivos en esquina

        # Filtro IR e histéresis frontal
        self._ir_lp: Optional[List[float]] = None
        self._alpha: float = float(av.get("iir_alpha", 0.20))
        self._front_on_th = float(self.IR_OBS_THRESHOLD)
        self._front_off_th = float(self.IR_OBS_THRESHOLD) * 0.7
        self._front_blocked_state = False

    # -----------------------------
    # Utilidades
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

    @staticmethod
    def _proj_s_on_line(x: float, y: float, x1: float, y1: float, x2: float, y2: float) -> float:
        vx, vy = x2 - x1, y2 - y1
        lx2 = vx * vx + vy * vy
        if lx2 <= 1e-9:
            return 0.0
        return ((x - x1) * vx + (y - y1) * vy) / math.sqrt(lx2)

    @staticmethod
    def _dist_to_line(x: float, y: float, x1: float, y1: float, x2: float, y2: float) -> float:
        vx, vy = x2 - x1, y2 - y1
        num = abs(vy * x - vx * y + x2 * y1 - y2 * x1)
        den = math.hypot(vx, vy)
        return num / max(den, 1e-9)

    def _clamp_delta_for_forward(self, base: float, raw_delta: float) -> float:
        max_delta = max(0.0, base - self.V_MIN)
        if raw_delta > max_delta:
            return max_delta
        if raw_delta < -max_delta:
            return -max_delta
        return raw_delta

    def _base_speed_from_ir(self, ir_vals: List[float]) -> float:
        if not ir_vals:
            return max(self.V_MIN, self.V_FWD)
        max_ir = max(ir_vals)
        if max_ir >= self.IR_CRITICAL_TH:
            return max(self.SLOW_MIN, self.V_MIN)
        if max_ir >= self.IR_SLOWDOWN_TH:
            return max(self.SLOW_MIN, self.SLOW_FACTOR_SLOW * self.V_FWD)
        if max_ir >= self.IR_FREE_TH:
            return max(self.V_MIN, self.SLOW_FACTOR_MED * self.V_FWD)
        return max(self.V_MIN, self.V_FWD)

    async def _apply(self, vl: float, vr: float) -> None:
        mx = max(abs(vl), abs(vr), 1e-9)
        if mx > self.MAX_WHEEL:
            scale = self.MAX_WHEEL / mx
            vl *= scale
            vr *= scale

        if self.safety and getattr(self.safety, "halted", None) and self.safety.halted.is_set():
            await self.robot.set_wheel_speeds(0, 0)
            if self.telemetry:
                self.telemetry.update_command(0.0, 0.0)
            await asyncio.sleep(0.05)
            return

        await self.robot.set_wheel_speeds(vl, vr)
        if self.telemetry:
            self.telemetry.update_command(vl, vr)

    def _front_center_value(self, ir_vals: List[float]) -> float:
        if not self.front_idx:
            return 0.0
        if 3 in self.front_idx and 3 < len(ir_vals):
            return ir_vals[3]
        best_i = min(self.front_idx, key=lambda i: abs(i - 3))
        return ir_vals[best_i] if best_i < len(ir_vals) else 0.0

    def _front_any_over(self, ir_vals: List[float], threshold: float) -> bool:
        for i in self.front_idx:
            if i < len(ir_vals) and ir_vals[i] > threshold:
                return True
        return False

    # -----------------------------
    # Selección inteligente de lado
    # -----------------------------
    def _choose_wall_side(self, left_val: float, right_val: float, goal_heading: float, current_heading: float) -> str:
        """
        Decide qué lado bordear considerando:
        1. Sensores IR (lado más libre)
        2. Geometría del objetivo (preferir lado que acerca al goal)
        
        Retorna: 'left' si obstáculo a la izquierda, 'right' si a la derecha
        """
        # Diferencia angular hacia el objetivo
        heading_err = _norm_deg(goal_heading - current_heading)
        
        # Diferencia de sensores (positivo => más obstáculo a la derecha)
        sensor_diff = right_val - left_val
        
        # Si diferencia de sensores es clara (>20), usar eso
        if abs(sensor_diff) > 20.0:
            if left_val < right_val:
                chosen = "right"  # obstáculo a la derecha, bordear por izquierda
                reason = f"sensor L={left_val:.0f} < R={right_val:.0f}"
            else:
                chosen = "left"   # obstáculo a la izquierda, bordear por derecha
                reason = f"sensor L={left_val:.0f} > R={right_val:.0f}"
        else:
            # Sensores similares: usar geometría del objetivo
            # Si el objetivo está a la derecha (heading_err > 0), preferir bordear con obstáculo a la IZQUIERDA
            # (para que al girar a la derecha en wall-follow, nos acerquemos al objetivo)
            if heading_err > 0:
                chosen = "left"
                reason = f"objetivo {heading_err:.1f}° derecha (sensores ~iguales)"
            else:
                chosen = "right"
                reason = f"objetivo {heading_err:.1f}° izquierda (sensores ~iguales)"
        
        print(f"  🧭 Lado elegido: obstáculo a la {chosen.upper()} ({reason})")
        return chosen

    # -----------------------------
    # Navegación principal
    # -----------------------------
    async def go_to(self, x_goal: float, y_goal: float, time_limit_s: Optional[float] = None):
        """
        Bug2 robusto: navega bordeando obstáculos SIN PERDER DE VISTA EL OBJETIVO.
        """
        t0 = time.monotonic()
        limit = float(time_limit_s or self.TIMEOUT_S)

        # Estado inicial
        x0, y0, th0 = await self._pose()
        self._mline = (x0, y0, x_goal, y_goal)
        self._hit = None
        self._s_hit = None
        self._wall_side = None
        self._front_blocked_state = False
        s_last = 0.0
        last_prog_t = t0

        STATE = "SEEK"
        await self.robot.set_lights_on_rgb(0, 128, 255)
        print(f"📍 Inicio: ({x0:.1f},{y0:.1f}) → Objetivo: ({x_goal:.1f},{y_goal:.1f})")

        while True:
            if (time.monotonic() - t0) > limit:
                await self._apply(0, 0)
                await self.robot.set_lights_on_rgb(255, 165, 0)
                x, y, th = await self._pose()
                print(f"⏱️ TIMEOUT tras {time.monotonic()-t0:.1f}s")
                return False, (x, y, th)

            # Pose y distancia al objetivo
            x, y, th = await self._pose()
            dx, dy = x_goal - x, y_goal - y
            dist = _hypot(dx, dy)
            
            if dist <= self.GOAL_TOL_CM:
                await self._apply(0, 0)
                await self.robot.set_lights_on_rgb(0, 255, 0)
                print(f"✅ OBJETIVO ALCANZADO en ({x:.1f},{y:.1f})")
                return True, (x, y, th)

            # Rumbo al objetivo
            goal_heading = math.degrees(math.atan2(dy, dx))
            heading_err = _norm_deg(goal_heading - th)

            # Geometría M-line
            x1, y1, x2, y2 = self._mline
            s_now = self._proj_s_on_line(x, y, x1, y1, x2, y2)
            d_to_line = self._dist_to_line(x, y, x1, y1, x2, y2)
            now = time.monotonic()

            # IR + histéresis
            ir = await self._ir_filtered()
            front_val = self._imax(ir, self.front_idx)
            left_val = self._imax(ir, self.left_idx)
            right_val = self._imax(ir, self.right_idx)

            if self._front_blocked_state:
                center_val = self._front_center_value(ir)
                self._front_blocked_state = center_val > self._front_off_th
            else:
                self._front_blocked_state = self._front_any_over(ir, self._front_on_th)
            front_blocked = self._front_blocked_state

            # Velocidad adaptativa
            base = self._base_speed_from_ir(ir)

            # ========== ESTADO: SEEK ==========
            if STATE == "SEEK":
                print(f"  🎯 SEEK: dist={dist:.1f}cm, err={heading_err:.1f}°, F={front_val:.0f} L={left_val:.0f} R={right_val:.0f}")
                
                # Control de rumbo
                raw_delta = self.KP_GOAL * heading_err
                delta = self._clamp_delta_for_forward(base, raw_delta)
                vl, vr = base - delta, base + delta

                if front_blocked:
                    # ===== OBSTÁCULO DETECTADO =====
                    await self._apply(0, 0)
                    await self.robot.wait(0.15)
                    
                    # Leer sensores frescos
                    ir_fresh = await self._ir_filtered()
                    left_fresh = self._imax(ir_fresh, self.left_idx)
                    right_fresh = self._imax(ir_fresh, self.right_idx)
                    
                    # Selección inteligente de lado
                    self._wall_side = self._choose_wall_side(left_fresh, right_fresh, goal_heading, th)
                    
                    # Memorizar hit
                    self._hit = (x, y, dist)
                    self._s_hit = s_now
                    
                    print(f"  🚧 HIT: ({x:.1f},{y:.1f}), s_hit={s_now:.1f}, dist_hit={dist:.1f}")
                    print(f"  🔄 Cambiando a WALL_FOLLOW (obstáculo a la {self._wall_side.upper()})")
                    
                    STATE = "WALL_FOLLOW"
                    await self.robot.wait(0.05)
                    continue
                else:
                    await self._apply(vl, vr)

            # ========== ESTADO: WALL_FOLLOW ==========
            elif STATE == "WALL_FOLLOW":
                # Control HÍBRIDO: seguir pared + bias hacia objetivo
                base_wf = max(base * 0.4, self.V_MIN)  # MUY LENTO para precisión
                
                # Componente 1: Control lateral (seguir pared)
                if self._wall_side == "left":
                    err_lat = left_val - (0.5 * self.IR_DIR_THRESHOLD)
                    lat_delta = self.KP_WALL * err_lat
                else:
                    err_lat = right_val - (0.5 * self.IR_DIR_THRESHOLD)
                    lat_delta = self.KP_WALL * err_lat
                
                # Componente 2: Bias hacia objetivo (30% del control de rumbo normal)
                goal_delta = 0.30 * self.KP_GOAL * heading_err
                
                # Combinar: pared + objetivo
                raw_delta = lat_delta + goal_delta
                delta = self._clamp_delta_for_forward(base_wf, raw_delta)
                
                if self._wall_side == "left":
                    vl, vr = base_wf + delta, base_wf - delta
                else:
                    vl, vr = base_wf - delta, base_wf + delta

                print(f"  🧱 WALL: d={dist:.1f}, s={s_now:.1f}, dLine={d_to_line:.1f}, err={heading_err:.1f}°, F={front_val:.0f} L={left_val:.0f} R={right_val:.0f}")

                if front_blocked:
                    # Esquina: estrategia de escape progresiva
                    await self._apply(0, 0)
                    await self.robot.wait(0.12)
                    
                    self._corner_turn_count += 1
                    
                    # Umbral dinámico: más permisivo después de múltiples intentos
                    escape_threshold = self._front_off_th * (1.0 + 0.5 * min(self._corner_turn_count, 3))
                    
                    turn_deg = 45.0
                    if self._wall_side == "left":
                        print(f"  ↪️  Esquina #{self._corner_turn_count}: girando DERECHA {turn_deg}°")
                        await self.robot.turn_right(turn_deg)
                    else:
                        print(f"  ↩️  Esquina #{self._corner_turn_count}: girando IZQUIERDA {turn_deg}°")
                        await self.robot.turn_left(turn_deg)
                    
                    await self.robot.wait(0.10)
                    
                    # Comprobar frontal post-giro
                    ir_post = await self._ir_filtered()
                    front_post = self._imax(ir_post, self.front_idx)
                    
                    # ESTRATEGIA DE ESCAPE:
                    # - Intento 1-2: buscar frontal < umbral
                    # - Intento 3+: FORZAR avance (escape agresivo)
                    if front_post < escape_threshold or self._corner_turn_count >= 3:
                        if self._corner_turn_count >= 3:
                            print(f"  ⚠️  ESCAPE FORZADO tras {self._corner_turn_count} giros (F={front_post:.0f})")
                        else:
                            print(f"  ➡️  Frontal aceptable ({front_post:.0f}<{escape_threshold:.0f}), avanzando 25cm")
                        
                        # Avanzar para salir de esquina
                        advance_base = max(self.V_MIN, base_wf * 1.2)
                        await self._apply(advance_base, advance_base)
                        await self.robot.wait(2.5)  # ~25cm
                        await self._apply(0, 0)
                        
                        # Reset contador
                        self._corner_turn_count = 0
                    else:
                        # Giro adicional (máximo 2 veces)
                        print(f"  🔄 Aún bloqueado ({front_post:.0f}>={escape_threshold:.0f}), girando más")
                        if self._wall_side == "left":
                            await self.robot.turn_right(45.0)
                        else:
                            await self.robot.turn_left(45.0)
                    
                    await self.robot.wait(0.10)
                    continue
                else:
                    # Frontal libre: reset contador y avanzar normalmente
                    self._corner_turn_count = 0
                    await self._apply(vl, vr)

                # ===== CONDICIÓN LEAVE (BUG2) =====
                # 1. Cruzar M-line (cerca de ella)
                # 2. Más cerca del objetivo que en hit
                # 3. Avanzado más allá del hit
                # 4. Ángulo razonable hacia objetivo
                # 5. Ventana refractaria OK
                
                closer_than_hit = (self._hit is None) or (dist < self._hit[2] - 3.0)
                progressed_past_hit = (self._s_hit is None) or (s_now > self._s_hit + 2.0)
                refract_ok = (now - self._last_leave_t) > self._leave_refract_s
                angle_ok = abs(heading_err) <= self.REACQUIRE_DEG

                if refract_ok and (d_to_line < 5.0) and closer_than_hit and progressed_past_hit and angle_ok:
                    print(f"  ✅ LEAVE: d_line={d_to_line:.1f}, dist={dist:.1f}<{self._hit[2]:.1f}, s={s_now:.1f}>{self._s_hit:.1f}, err={heading_err:.1f}°")
                    STATE = "SEEK"
                    self._last_leave_t = now
                    await self.robot.wait(0.05)
                    continue

            await self.robot.wait(0.06)

    # ========== MODO CRUISE (sin objetivo) ==========
    async def cruise(self, time_limit_s: Optional[float] = None):
        """Modo crucero sin objetivo específico."""
        t0 = time.monotonic()
        limit = float(time_limit_s or self.TIMEOUT_S)
        await self.robot.set_lights_on_rgb(255, 255, 255)
        self._front_blocked_state = False

        while True:
            if (time.monotonic() - t0) > limit:
                await self._apply(0, 0)
                x, y, th = await self._pose()
                return False, (x, y, th)

            ir = await self._ir_filtered()
            base = self._base_speed_from_ir(ir)
            left_val = self._imax(ir, self.left_idx)
            right_val = self._imax(ir, self.right_idx)

            if self._front_blocked_state:
                center_val = self._front_center_value(ir)
                self._front_blocked_state = center_val > self._front_off_th
            else:
                self._front_blocked_state = self._front_any_over(ir, self._front_on_th)

            if self._front_blocked_state:
                await self._apply(0, 0)
                await self.robot.wait(0.15)
                if left_val < right_val:
                    await self.robot.turn_left(45.0)
                else:
                    await self.robot.turn_right(45.0)
                await self.robot.wait(0.10)
            else:
                raw_delta = self.KP_WALL * (right_val - left_val)
                delta = self._clamp_delta_for_forward(base, raw_delta)
                vl, vr = base - delta, base + delta
                await self._apply(vl, vr)

            await self.robot.wait(0.06)
