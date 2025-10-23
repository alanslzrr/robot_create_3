# core/telemetry.py
# Telemetría a ~10 Hz: pose, velocidades comandadas, batería, IR, bumps, acc RMS (si disponible)
# El app debe llamar a update_command(vl, vr) para que quede registrado lo que se manda a ruedas.

import csv, os, time, math, asyncio
from datetime import datetime

class TelemetryLogger:
    def __init__(self, robot, out_dir="nodes/logs", period_s=0.1):
        self.robot = robot
        self.out_dir = out_dir
        self.period_s = period_s
        self._task = None
        self._cmd_vl = 0.0
        self._cmd_vr = 0.0
        self._path = None
        os.makedirs(self.out_dir, exist_ok=True)

    def update_command(self, vl: float, vr: float):
        self._cmd_vl, self._cmd_vr = float(vl), float(vr)

    async def start(self):
        if self._task is None:
            self._path = os.path.join(
                self.out_dir,
                f"telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            )
            write_header = not os.path.exists(self._path)
            if write_header:
                with open(self._path, "w", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    w.writerow(["ts","x","y","theta","cmd_vl","cmd_vr",
                                "acc_rms","ir_max","bumps","batt"])
            self._task = asyncio.create_task(self._run())

    async def stop(self):
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except Exception:
                pass
            self._task = None

    async def _run(self):
        while True:
            try:
                # Pose
                x=y=theta=None
                try:
                    p = await self.robot.get_position()
                    x = getattr(p, "x", None) if hasattr(p, "x") else (p[0] if isinstance(p,(list,tuple)) else None)
                    y = getattr(p, "y", None) if hasattr(p, "y") else (p[1] if isinstance(p,(list,tuple)) else None)
                    theta = getattr(p, "heading", None) if hasattr(p, "heading") else (p[2] if isinstance(p,(list,tuple)) else None)
                except Exception:
                    pass

                # Acc RMS
                acc_rms = None
                try:
                    acc = await self.robot.get_accelerometer()
                    ax = getattr(acc, "x", 0.0); ay = getattr(acc, "y", 0.0); az = getattr(acc, "z", 0.0)
                    acc_rms = math.sqrt(ax*ax + ay*ay + az*az)
                except Exception:
                    pass

                # IR max (valor más alto = obstáculo más cerca)
                ir_max = None
                try:
                    sensors = (await self.robot.get_ir_proximity()).sensors
                    vals = [s for s in sensors if s is not None and s >= 0]
                    if vals:
                        ir_max = max(vals)
                except Exception:
                    pass

                # Bumps - retorna tupla (left, right)
                bumps = 0
                try:
                    left_bump, right_bump = await self.robot.get_bumpers()
                    if left_bump: bumps += 1
                    if right_bump: bumps += 1
                except Exception:
                    pass

                # Batt
                batt = None
                try:
                    batt = await self.robot.get_battery_level()
                except Exception:
                    pass

                row = [
                    time.strftime("%Y-%m-%dT%H:%M:%S"),
                    x, y, theta,
                    self._cmd_vl, self._cmd_vr,
                    acc_rms, ir_max, bumps, batt
                ]
                with open(self._path, "a", newline="", encoding="utf-8") as f:
                    csv.writer(f).writerow(row)

                await asyncio.sleep(self.period_s)
            except asyncio.CancelledError:
                break
            except Exception:
                await asyncio.sleep(self.period_s)
