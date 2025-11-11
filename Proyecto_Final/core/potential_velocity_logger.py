"""Velocity logger adaptado de PL4/src/velocity_logger.py."""

import csv
import time
from pathlib import Path
from datetime import datetime


class VelocityLogger:
    def __init__(self, potential_type='linear', log_dir='nodes/logs/potential'):
        self.potential_type = potential_type
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"velocities_{potential_type}_{timestamp}.csv"
        self.filepath = self.log_dir / filename
        self.file = None
        self.writer = None
        self.start_time = None

    def start(self):
        self.file = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'timestamp', 'elapsed_s',
            'x_cm', 'y_cm', 'theta_deg',
            'distance_cm', 'v_left', 'v_right',
            'v_linear', 'omega', 'angle_error_deg',
            'fx_repulsive', 'fy_repulsive', 'num_obstacles',
            'potential_type'
        ])
        self.start_time = time.time()
        print(f"✅ Velocity logger iniciado: {self.filepath}")

    def log(self, position, distance, v_left, v_right, info):
        if not self.writer:
            return
        elapsed = time.time() - self.start_time
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.writer.writerow([
            timestamp,
            f"{elapsed:.3f}",
            f"{position['x']:.2f}",
            f"{position['y']:.2f}",
            f"{position['theta']:.2f}",
            f"{distance:.2f}",
            f"{v_left:.2f}",
            f"{v_right:.2f}",
            f"{info.get('v_linear', 0):.2f}",
            f"{info.get('omega', 0):.3f}",
            f"{info.get('angle_error_deg', 0):.2f}",
            f"{info.get('fx_repulsive', 0):.2f}",
            f"{info.get('fy_repulsive', 0):.2f}",
            info.get('num_obstacles', 0),
            info.get('potential_type', self.potential_type)
        ])

    def stop(self):
        if self.file:
            self.file.close()
            print(f"📊 Log guardado: {self.filepath}")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
