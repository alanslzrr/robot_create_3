"""
Sistema de registro de datos para análisis de funciones de potencial atractivo

===============================================================================
INFORMACIÓN DEL PROYECTO
===============================================================================

Autores:
    - Alan Ariel Salazar
    - Yago Ramos Sánchez

Institución:
    Universidad Intercontinental de la Empresa (UIE)

Profesor:
    Eladio Dapena

Asignatura:
    Robots Autónomos

Fecha de Finalización:
    6 de noviembre de 2025

Robot SDK:
    irobot-edu-sdk

===============================================================================
OBJETIVO GENERAL
===============================================================================

Crear un sistema de registro de datos que capture todas las variables relevantes
durante la navegación en archivos CSV, permitiendo análisis posterior y comparación
cuantitativa entre diferentes funciones de potencial y facilitando la evaluación
objetiva del desempeño de cada función.

===============================================================================
OBJETIVOS ESPECÍFICOS
===============================================================================

1. Registrar todas las variables relevantes de navegación en formato CSV estándar
   que pueda ser procesado fácilmente por herramientas de análisis

2. Incluir información suficiente para calcular métricas de desempeño como tiempo
   total, precisión, eficiencia de trayectoria y características de velocidad

3. Generar archivos con nombres únicos que incluyan timestamp y tipo de potencial
   para facilitar la organización y identificación de ejecuciones

4. Proporcionar una interfaz simple que pueda iniciarse al comienzo de la navegación
   y detenerse al finalizar, registrando datos en cada iteración del bucle de control

5. Soportar tanto navegación con potencial atractivo puro como combinado con repulsivo,
   incluyendo información adicional cuando está disponible

Comportamiento esperado:
    - Crear archivo CSV único por ejecución con timestamp en el nombre
    - Registrar 12 columnas de datos por cada iteración del control
    - Escribir en buffer y cerrar archivo al finalizar navegación
    - Permitir análisis comparativo posterior con analyze_results.py
    - Incluir metadato del tipo de potencial para identificación

Clase principal:
    
    VelocityLogger:
        Gestiona creación, escritura y cierre de archivos CSV de telemetría.
        
        Métodos:
            __init__(potential_type='linear', log_dir='logs'):
                Inicializa logger con tipo de potencial y directorio de salida.
                
                Parámetros:
                    potential_type: Tipo de potencial ('linear', 'quadratic', 'conic', 'exponential')
                    log_dir: Directorio donde guardar CSVs (se crea si no existe)
            
            start():
                Crea archivo CSV con timestamp y escribe cabecera con 12 columnas:
                - timestamp: Tiempo desde inicio en segundos
                - x_cm, y_cm, heading_deg: Posición odométrica
                - v_left_cm_s, v_right_cm_s: Velocidades individuales de ruedas
                - v_linear_cm_s: Velocidad lineal resultante
                - omega_deg_s: Velocidad angular
                - distance_error_cm: Error de distancia al objetivo
                - heading_error_deg: Error angular hacia objetivo
                - potential_type: Tipo de función de potencial
                
                Retorna:
                    Path del archivo CSV creado
            
            log(timestamp, x, y, heading, v_left, v_right, v_linear, omega, dist_err, heading_err):
                Escribe una fila de datos en el CSV.
                
                Parámetros:
                    timestamp: Tiempo en segundos desde inicio
                    x, y: Posición en cm
                    heading: Orientación en grados
                    v_left, v_right: Velocidades de ruedas en cm/s
                    v_linear: Velocidad lineal en cm/s
                    omega: Velocidad angular en deg/s
                    dist_err: Error de distancia en cm
                    heading_err: Error angular en grados
            
            stop():
                Cierra archivo CSV de forma segura y limpia.

Formato de archivo:
    Nombre: velocity_log_{potential_type}_YYYYMMDD_HHMMSS.csv
    Ubicación: {log_dir}/
    Columnas: 12 (ver cabecera en start())
    Separador: coma (,)

Uso típico:
    logger = VelocityLogger(potential_type='quadratic')
    csv_path = logger.start()
    # ... bucle de navegación ...
    logger.log(t, x, y, θ, vL, vR, v, ω, d_err, θ_err)
    logger.stop()
"""

import csv
import time
from pathlib import Path
from datetime import datetime


class VelocityLogger:
    """Logger para análisis comparativo de funciones de potencial"""
    
    def __init__(self, potential_type='linear', log_dir='logs'):
        self.potential_type = potential_type
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"velocities_{potential_type}_{timestamp}.csv"
        self.filepath = self.log_dir / filename
        
        self.file = None
        self.writer = None
        self.start_time = None
        
    def start(self):
        """Inicia el logger y crea el archivo CSV"""
        self.file = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.file)
        
        # Header con columnas adicionales para potencial repulsivo
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
        """
        Registra una entrada de velocidad
        
        Args:
            position: dict con 'x', 'y', 'theta'
            distance: distancia a meta (cm)
            v_left: velocidad rueda izquierda (cm/s)
            v_right: velocidad rueda derecha (cm/s)
            info: dict con info adicional del potencial (incluyendo repulsivo)
        """
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
        """Cierra el archivo"""
        if self.file:
            self.file.close()
            print(f"📊 Log guardado: {self.filepath}")
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()