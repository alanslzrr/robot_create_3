"""
Sistema de monitoreo en tiempo real de sensores del iRobot Create 3

Autores: Alan Salazar, Yago Ramos
Fecha: 4 de noviembre de 2025
Institución: UIE Universidad Intercontinental de la Empresa
Asignatura: Robots Autónomos - Profesor Eladio Dapena
Robot SDK: irobot-edu-sdk

OBJETIVOS PRINCIPALES:

En este módulo implementamos un sistema de monitoreo asíncrono que proporciona
visualización continua del estado de todos los sensores del robot durante la
navegación autónoma. Nuestro objetivo principal era crear una herramienta de
depuración y monitoreo que permitiera observar el comportamiento del robot en
tiempo real sin interferir con el bucle de control principal.

Los objetivos específicos que buscamos alcanzar incluyen:

1. Implementar un sistema de logging asíncrono que funcione en segundo plano
   sin bloquear el bucle principal de navegación
2. Proporcionar visualización periódica de todos los sensores críticos del robot
   incluyendo sensores IR, bumpers, odometría y nivel de batería
3. Integrar análisis de seguridad que muestre el nivel de peligro actual según
   las lecturas de sensores IR frontales
4. Proporcionar una función de instantánea de sensores para capturas puntuales
   sin iniciar el logger completo
5. Garantizar que el sistema pueda iniciarse y detenerse de forma limpia sin
   dejar tareas pendientes

Comportamiento esperado:
    - Iniciar tarea asíncrona en segundo plano al crear instancia
    - Imprimir estado de sensores cada LOG_INTERVAL_S (configurable, default 1s)
    - Mostrar valores IR de los 7 sensores con formato legible
    - Indicar estado de bumpers izquierdo/derecho
    - Mostrar posición odométrica (x, y, θ) en cm y grados
    - Reportar nivel de batería en porcentaje
    - Permitir cierre limpio de la tarea al finalizar navegación

Clase principal:
    
    SensorLogger:
        Gestiona impresión periódica de sensores mediante asyncio.
        
        Métodos:
            __init__(robot, interval=None):
                Inicializa logger con referencia al robot y arranca tarea asíncrona.
                
                Parámetros:
                    robot: Instancia de irobot_edu_sdk.backend.Create3
                    interval: Intervalo de impresión en segundos (default: LOG_INTERVAL_S)
            
            _print_sensors():
                Bucle asíncrono que lee sensores cada interval e imprime:
                - IR[0-6]: Valores de proximidad infrarroja (0-4095)
                - Bumpers: Estado binario izquierdo/derecho
                - Posición: (x, y, θ) de odometría
                - Batería: Porcentaje actual
                
                Ejecuta indefinidamente hasta cancelación externa.
            
            stop():
                Cancela la tarea asíncrona de forma segura para detener logging.

Formato de salida (cada interval):
    === Sensores ===
    IR: [123, 45, 67, 89, 12, 34, 56]
    Bumpers: (False, False)
    Posición: (x=50.2cm, y=30.1cm, θ=45.3°)
    Batería: 85%

Configuración:
    LOG_INTERVAL_S: Intervalo entre impresiones (config.py)

Uso típico:
    logger = SensorLogger(robot)  # Usa intervalo por defecto
    # ... navegación ...
    logger.stop()  # Al finalizar
"""

import asyncio
from . import config


class SensorLogger:
    """
    Logger asíncrono de sensores que imprime información cada LOG_INTERVAL_S
    """
    
    def __init__(self, robot, interval=None):
        """
        Args:
            robot: Instancia del Create3
            interval: Intervalo de logging en segundos (usa config si es None)
        """
        self.robot = robot
        self.interval = interval or config.LOG_INTERVAL_S
        self.running = False
        self.task = None
    
    async def _log_loop(self):
        """Bucle interno de logging"""
        while self.running:
            try:
                await self._print_sensors()
            except Exception as e:
                print(f"⚠️  Error en logger: {e}")
            
            await self.robot.wait(self.interval)
    
    async def _print_sensors(self):
        """Imprime todos los sensores de forma organizada"""
        # Leer sensores
        pos = await self.robot.get_position()
        ir_prox = await self.robot.get_ir_proximity()
        bumpers = await self.robot.get_bumpers()
        battery_mv, battery_pct = await self.robot.get_battery_level()
        
        ir_sensors = ir_prox.sensors if hasattr(ir_prox, 'sensors') else ir_prox
        
        # Formato compacto y legible
        print("\n" + "="*60)
        print("📊 SENSORES")
        print("="*60)
        
        # Posición
        print(f"📍 Posición: x={pos.x:7.2f} cm  y={pos.y:7.2f} cm  θ={pos.heading:6.1f}°")
        
        # IR
        if ir_sensors and len(ir_sensors) >= 7:
            print(f"IR: ", end="")
            for i in config.IR_INDICES:
                print(f"[{i}]={ir_sensors[i]:4d} ", end="")
            print()
            
            # Análisis de seguridad (sensores frontales críticos: 1,2,3,4)
            max_front = max(ir_sensors[1], ir_sensors[2], ir_sensors[3], ir_sensors[4])
            
            # Determinar nivel de seguridad
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
        
        # Bumpers
        bump_left, bump_right = bumpers
        bump_status = ""
        if bump_left and bump_right:
            bump_status = "⚠️ COLISIÓN AMBOS"
        elif bump_left:
            bump_status = "⚠️ COLISIÓN IZQ"
        elif bump_right:
            bump_status = "⚠️ COLISIÓN DER"
        else:
            bump_status = "✅ Sin colisión"
        
        print(f"🛡️  Bumpers: L={bump_left}  R={bump_right}  {bump_status}")
        
        # Batería
        print(f"🔋 Batería: {battery_pct}% ({battery_mv} mV)")
        print("="*60)
    
    def start(self):
        """Inicia el logger en background"""
        if not self.running:
            self.running = True
            self.task = asyncio.create_task(self._log_loop())
            print("✅ Logger de sensores iniciado")
    
    def stop(self):
        """Detiene el logger"""
        self.running = False
        if self.task:
            self.task.cancel()
        print("🛑 Logger de sensores detenido")


async def get_sensor_snapshot(robot):
    """
    Obtiene una instantánea de todos los sensores sin imprimir.
    
    Returns:
        dict con 'position', 'ir_sensors', 'bumpers', 'battery'
    """
    pos = await robot.get_position()
    ir_prox = await robot.get_ir_proximity()
    bumpers = await robot.get_bumpers()
    battery_mv, battery_pct = await robot.get_battery_level()
    
    ir_sensors = ir_prox.sensors if hasattr(ir_prox, 'sensors') else ir_prox
    
    return {
        'position': {'x': pos.x, 'y': pos.y, 'theta': pos.heading},
        'ir_sensors': list(ir_sensors) if ir_sensors else [],
        'bumpers': bumpers,
        'battery': {'percent': battery_pct, 'millivolts': battery_mv}
    }
