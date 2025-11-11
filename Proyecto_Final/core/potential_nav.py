import math
import asyncio
from typing import Optional, Tuple

from . import potential_config as config
from .potential_fields import combined_potential_speeds, POTENTIAL_TYPES, reset_velocity_ramp
from .potential_velocity_logger import VelocityLogger
from .potential_sensor_logger import SensorLogger
from .potential_safety import saturate_wheel_speeds, emergency_stop_needed


class CombinedPotentialNavigator:
    """
    Navegación combinada (atractivo + repulsivo) adaptada de PL4.
    """

    def __init__(
        self,
        robot,
        q_initial: Tuple[float, float, float],
        q_goal: Tuple[float, float],
        potential_type: str = "linear",
        k_rep: Optional[float] = None,
        d_influence: Optional[float] = None,
        telemetry=None,
        safety=None,
        debug: bool = False,
        log_dir: str = "nodes/logs/potential",
    ) -> None:
        self.robot = robot
        self.q_initial = q_initial
        self.q_goal = q_goal
        self.potential_type = potential_type if potential_type in POTENTIAL_TYPES else "linear"
        self.k_rep = k_rep or config.K_REPULSIVE
        self.d_influence = d_influence or config.D_INFLUENCE
        self.telemetry = telemetry
        self.safety = safety
        self.debug = debug

        self.vel_logger = VelocityLogger(f"{self.potential_type}_combined", log_dir=log_dir)
        self.logger = SensorLogger(
            robot,
            position_offset_x=q_initial[0],
            position_offset_y=q_initial[1],
            heading_offset=0,
        )
        self.running = False
        self.current_led_color = None
        self.obstacle_detected = False
        self.max_collisions = 5

        self.position_offset_x = q_initial[0]
        self.position_offset_y = q_initial[1]
        self.initial_heading = q_initial[2]
        self.odometry_to_world_rotation = 0.0
        self.reset_heading = 0.0
        self.heading_offset = 0.0

    async def _set_led(self, r: int, g: int, b: int, tag: str):
        if self.current_led_color != tag:
            try:
                await self.robot.set_lights_rgb(r, g, b)
            except Exception:
                pass
            self.current_led_color = tag

    def _apply_transform(self, pos):
        odom_x = pos.x if hasattr(pos, "x") else pos[0]
        odom_y = pos.y if hasattr(pos, "y") else pos[1]
        odom_heading = pos.heading if hasattr(pos, "heading") else pos[2]

        cos_rot = math.cos(self.odometry_to_world_rotation)
        sin_rot = math.sin(self.odometry_to_world_rotation)

        rotated_x = odom_x * cos_rot - odom_y * sin_rot
        rotated_y = odom_x * sin_rot + odom_y * cos_rot

        actual_x = rotated_x + self.position_offset_x
        actual_y = rotated_y + self.position_offset_y
        actual_heading = odom_heading + self.heading_offset

        while actual_heading > 180:
            actual_heading -= 360
        while actual_heading <= -180:
            actual_heading += 360

        return actual_x, actual_y, actual_heading

    async def _apply_wheels(self, v_left: float, v_right: float):
        v_left, v_right = saturate_wheel_speeds(v_left, v_right)

        if self.safety and getattr(self.safety, "halted", None) and self.safety.halted.is_set():
            await self.robot.set_wheel_speeds(0, 0)
            if self.telemetry:
                self.telemetry.update_command(0.0, 0.0)
            await asyncio.sleep(0.05)
            return

        await self.robot.set_wheel_speeds(v_left, v_right)
        if self.telemetry:
            self.telemetry.update_command(v_left, v_right)

    async def navigate(self) -> bool:
        await self.robot.reset_navigation()
        pos_initial = await self.robot.get_position()
        self.reset_heading = pos_initial.heading if hasattr(pos_initial, "heading") else pos_initial[2]
        desired_heading = self.q_initial[2]
        self.odometry_to_world_rotation = math.radians(desired_heading - self.reset_heading)
        self.heading_offset = desired_heading - self.reset_heading
        self.logger.heading_offset = self.heading_offset

        reset_velocity_ramp()
        await self._set_led(0, 255, 0, "green")
        await self.robot.wait(1.0)

        self.logger.start()
        self.vel_logger.start()
        self.running = True
        iteration = 0
        collision_count = 0

        try:
            while self.running:
                iteration += 1
                pos = await self.robot.get_position()
                if pos is None:
                    await self.robot.wait(0.05)
                    continue

                actual_x, actual_y, actual_heading = self._apply_transform(pos)
                q = (actual_x, actual_y, actual_heading)

                dx = self.q_goal[0] - actual_x
                dy = self.q_goal[1] - actual_y
                distance = math.hypot(dx, dy)

                if distance < max(config.TOL_DIST_CM, 10.0):
                    await self.robot.set_wheel_speeds(0, 0)
                    if self.telemetry:
                        self.telemetry.update_command(0.0, 0.0)
                    await self._set_led(0, 255, 0, "green")
                    try:
                        await self.robot.play_note(80, 0.2)
                    except Exception:
                        pass
                    return True

                ir_prox = await self.robot.get_ir_proximity()
                ir_sensors = ir_prox.sensors if hasattr(ir_prox, "sensors") else []
                bumpers = await self.robot.get_bumpers()

                if iteration <= 3 and self.debug:
                    print(f"[DEBUG {iteration}] q={q} goal={self.q_goal} distance={distance:.2f}")

                v_left, v_right, _, info = combined_potential_speeds(
                    q,
                    self.q_goal,
                    ir_sensors=ir_sensors,
                    k_lin=None,
                    k_ang=None,
                    k_rep=self.k_rep,
                    d_influence=self.d_influence,
                    potential_type=self.potential_type,
                )

                self.vel_logger.log(
                    {"x": pos.x, "y": pos.y, "theta": pos.heading},
                    distance,
                    v_left,
                    v_right,
                    info,
                )

                if emergency_stop_needed(bumpers):
                    collision_count += 1
                    await self.robot.set_wheel_speeds(0, 0)
                    if self.telemetry:
                        self.telemetry.update_command(0.0, 0.0)
                    print(f"[COLLISION] {collision_count}/{self.max_collisions}")
                    if collision_count >= self.max_collisions:
                        return False
                    await self.robot.set_wheel_speeds(-10, -10)
                    await self.robot.wait(1.0)
                    await self.robot.set_wheel_speeds(0, 0)
                    await self.robot.wait(0.5)
                    continue

                num_obstacles = info.get("num_obstacles", 0)
                safety_level = info.get("safety_level", "CLEAR")
                max_ir_all = info.get("max_ir_all", 0.0)

                if num_obstacles > 0 and max_ir_all >= config.IR_THRESHOLD_CAUTION:
                    if max_ir_all >= config.IR_THRESHOLD_WARNING:
                        await self._set_led(0, 255, 255, "cyan")
                    else:
                        await self._set_led(255, 165, 0, "orange")
                        if not self.obstacle_detected:
                            try:
                                await self.robot.play_note(440, 0.2)
                            except Exception:
                                pass
                            self.obstacle_detected = True
                else:
                    await self._set_led(0, 0, 255, "blue")
                    self.obstacle_detected = False

                if self.debug and iteration % 10 == 0:
                    print(
                        f"[{iteration:04d}] d={distance:5.1f} "
                        f"obs={num_obstacles} level={safety_level} "
                        f"F_rep=({info.get('fx_repulsive', 0):6.1f},{info.get('fy_repulsive', 0):6.1f}) "
                        f"vL={v_left:5.1f} vR={v_right:5.1f}"
                    )

                await self._apply_wheels(v_left, v_right)
                await self.robot.wait(config.CONTROL_DT)

        except Exception as exc:
            print(f"[ERROR] Navegación potencial falló: {exc}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            self.running = False
            try:
                await self.robot.set_wheel_speeds(0, 0)
            except Exception:
                pass
            if self.telemetry:
                self.telemetry.update_command(0.0, 0.0)
            self.logger.stop()
            self.vel_logger.stop()

        return False


__all__ = ["CombinedPotentialNavigator", "POTENTIAL_TYPES"]

