# core/safety.py
# Safety no intrusivo con override inmediato
# Basado en valores probados de PL2/examples

import asyncio
from typing import Optional

class SafetyMonitorV2:
    """
    Monitor de seguridad no intrusivo:
    - Por defecto DESACTIVADO en teleop manual
    - Señal visual + frenado momentáneo (no bloquea)
    - Override inmediato con comando 'clear'
    - Valores de sensores alineados con PL2/examples
    """
    
    def __init__(self, robot, ir_threshold: int = 120, period_s: float = 0.1, front_idx=(2, 3, 4)):
        self.robot = robot
        self.ir_threshold = ir_threshold  # Valor de PL2 (120 = ~15 cm)
        self.period_s = period_s
        self.front_idx = tuple(front_idx)
        self._task: Optional[asyncio.Task] = None
        self.halted = asyncio.Event()
        self.enabled = False  # Por defecto DESACTIVADO
        self.override_count = 0  # Contador de overrides
        
    async def start(self):
        """Inicia el monitor (si está habilitado)"""
        if self._task is None and self.enabled:
            self._task = asyncio.create_task(self._run())
    
    async def stop(self):
        """Detiene el monitor"""
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
    
    def enable(self, on: bool = True):
        """Habilita/deshabilita el monitor"""
        self.enabled = on
        if not on and self._task:
            asyncio.create_task(self.stop())
        elif on and self._task is None:
            asyncio.create_task(self.start())
    
    async def _run(self):
        """Loop principal del monitor (solo si está habilitado)"""
        while self.enabled:
            try:
                hazard = False
                
                # 1) IR proximidad - valores ALTOS = obstáculo cerca (PL2)
                try:
                    sensors = (await self.robot.get_ir_proximity()).sensors
                    # Usar SOLO índices frontales configurados (p.ej., 2,3,4)
                    front_vals = [sensors[i] for i in self.front_idx if i < len(sensors)]
                    if any((s is not None and s > self.ir_threshold) for s in front_vals):
                        hazard = True
                except Exception:
                    pass
                
                # 2) Bumpers - tupla (left, right)
                try:
                    left_bump, right_bump = await self.robot.get_bumpers()
                    if left_bump or right_bump:
                        hazard = True
                except Exception:
                    pass
                
                # 3) Cliff - lista de valores
                try:
                    cliff_sensors = await self.robot.get_cliff_sensors()
                    if any(c > 50 for c in cliff_sensors if c is not None):
                        hazard = True
                except Exception:
                    pass
                
                # Acción de seguridad (NO BLOQUEANTE)
                if hazard and not self.halted.is_set():
                    await self._safety_action()
                
                await asyncio.sleep(self.period_s)
            except asyncio.CancelledError:
                break
            except Exception:
                await asyncio.sleep(self.period_s)
    
    async def _safety_action(self):
        """Acción de seguridad: frenado + señal visual (no bloquea)"""
        # Frenado momentáneo
        await self.robot.set_wheel_speeds(0, 0)
        
        # Señal visual (parpadeo rojo)
        for _ in range(3):
            await self.robot.set_lights_on_rgb(255, 0, 0)  # Rojo
            await asyncio.sleep(0.1)
            await self.robot.set_lights_on_rgb(0, 0, 0)    # Apagado
            await asyncio.sleep(0.1)
        
        # Marcar como detenido
        self.halted.set()
        print("⚠️  Safety: obstáculo detectado (usa 'clear' para continuar)")
    
    async def clear_halt(self):
        """Override inmediato - NO BLOQUEANTE"""
        if self.halted.is_set():
            self.halted.clear()
            self.override_count += 1
            await self.robot.set_lights_on_rgb(0, 255, 0)  # Verde
            print(f"✓ Safety: override #{self.override_count} - puedes continuar")
    
    async def brake(self):
        """Freno manual inmediato"""
        await self.robot.set_wheel_speeds(0, 0)
        await self.robot.set_lights_on_rgb(255, 0, 0)
        self.halted.set()
        print("🛑 Freno manual activado")
    
    def get_status(self) -> dict:
        """Estado del monitor para diagnóstico"""
        return {
            "enabled": self.enabled,
            "halted": self.halted.is_set(),
            "ir_threshold": self.ir_threshold,
            "front_idx": list(self.front_idx),
            "override_count": self.override_count,
            "running": self._task is not None and not self._task.done()
        }
