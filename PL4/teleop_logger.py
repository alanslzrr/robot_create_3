"""
Logger + tele-operación reutilizable.
Se importa desde PRM01_P01 (no se ejecuta stand-alone).
"""
import asyncio, time
from irobot_edu_sdk.robots import Create3

IR_IDXS = list(range(7))              # 0-6
LOG_DT  = 1.0                         # s

async def log_sensors(robot):
    while True:
        prox  = (await robot.get_ir_proximity()).sensors
        bumps = await robot.get_bumpers()
        print("IR:", {i:prox[i] for i in IR_IDXS}, "| Bumpers:", bumps)
        await robot.wait(LOG_DT)

def start_logger(robot):
    asyncio.create_task(log_sensors(robot))
