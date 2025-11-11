"""Sensor logger adaptado de PL4/src/sensor_logger.py."""

import asyncio
from . import potential_config as config


class SensorLogger:
    def __init__(self, robot, interval=None, position_offset_x=0, position_offset_y=0, heading_offset=0):
        self.robot = robot
        self.interval = interval or config.LOG_INTERVAL_S
        self.position_offset_x = position_offset_x
        self.position_offset_y = position_offset_y
        self.heading_offset = heading_offset
        self.running = False
        self.task = None

    async def _log_loop(self):
        while self.running:
            try:
                await self._print_sensors()
            except Exception as exc:
                print(f"⚠️  Error en logger: {exc}")
            await self.robot.wait(self.interval)

    async def _print_sensors(self):
        pos = await self.robot.get_position()
        ir_prox = await self.robot.get_ir_proximity()
        bumpers = await self.robot.get_bumpers()
        battery_mv, battery_pct = await self.robot.get_battery_level()

        actual_x = pos.x + self.position_offset_x
        actual_y = pos.y + self.position_offset_y
        actual_heading = pos.heading + self.heading_offset
        while actual_heading > 180:
            actual_heading -= 360
        while actual_heading <= -180:
            actual_heading += 360

        ir_sensors = ir_prox.sensors if hasattr(ir_prox, 'sensors') else ir_prox

        print("\n" + "=" * 60)
        print("📊 SENSORES")
        print("=" * 60)
        print(f"📍 Posición: x={actual_x:7.2f} cm  y={actual_y:7.2f} cm  θ={actual_heading:6.1f}°")

        if ir_sensors and len(ir_sensors) >= 7:
            print("IR: ", end="")
            for i in config.IR_INDICES:
                print(f"[{i}]={ir_sensors[i]:4d} ", end="")
            print()
            max_front = max(ir_sensors[1], ir_sensors[2], ir_sensors[3], ir_sensors[4])
            if max_front >= config.IR_THRESHOLD_EMERGENCY:
                status = "🚨 EMERGENCIA"
                v_limit = f"v≤{config.V_MAX_EMERGENCY}cm/s"
            elif max_front >= config.IR_THRESHOLD_CRITICAL:
                status = "🔴 CRÍTICO"
                v_limit = f"v≤{config.V_MAX_CRITICAL}cm/s"
            elif max_front >= config.IR_THRESHOLD_WARNING:
                status = "⚠️  ADVERTENCIA"
                v_limit = f"v≤{config.V_MAX_WARNING}cm/s"
            elif max_front >= config.IR_THRESHOLD_CAUTION:
                status = "⚡ PRECAUCIÓN"
                v_limit = f"v≤{config.V_MAX_CAUTION}cm/s"
            else:
                status = "✅ Libre"
                v_limit = f"v≤{config.V_MAX_CM_S}cm/s"
            print(f"   Max frontal: {max_front:4d}  {status}  ({v_limit})")

        bump_left, bump_right = bumpers
        if bump_left and bump_right:
            bump_status = "⚠️ COLISIÓN AMBOS"
        elif bump_left:
            bump_status = "⚠️ COLISIÓN IZQ"
        elif bump_right:
            bump_status = "⚠️ COLISIÓN DER"
        else:
            bump_status = "✅ Sin colisión"
        print(f"🛡️  Bumpers: L={bump_left}  R={bump_right}  {bump_status}")
        print(f"🔋 Batería: {battery_pct}% ({battery_mv} mV)")
        print("=" * 60)

    def start(self):
        if not self.running:
            self.running = True
            self.task = asyncio.create_task(self._log_loop())
            print("✅ Logger de sensores iniciado")

    def stop(self):
        self.running = False
        if self.task:
            self.task.cancel()
        print("🛑 Logger de sensores detenido")
