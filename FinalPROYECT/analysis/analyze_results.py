#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Herramienta de análisis comparativo para funciones de potencial atractivo

Autores: Alan Salazar, Yago Ramos
Fecha: 4 de noviembre de 2025
Institución: UIE Universidad Intercontinental de la Empresa
Asignatura: Robots Autónomos - Profesor Eladio Dapena
Robot SDK: irobot-edu-sdk (análisis offline)

OBJETIVOS PRINCIPALES:

En este módulo implementamos una herramienta de análisis que procesa los archivos
CSV generados durante las ejecuciones de navegación y calcula métricas cuantitativas
que permiten comparar objetivamente el desempeño de diferentes funciones de potencial.
Nuestro objetivo principal era crear una herramienta que facilitara la evaluación
de qué función de potencial ofrece mejores resultados según diferentes criterios.

Los objetivos específicos que buscamos alcanzar incluyen:

1. Procesar automáticamente todos los archivos CSV generados por velocity_logger.py
   sin necesidad de configuración manual
2. Calcular métricas relevantes de desempeño como tiempo total, precisión final,
   distancia recorrida y características de velocidad
3. Generar tablas comparativas formateadas que faciliten la visualización de
   diferencias entre funciones de potencial
4. Identificar automáticamente la mejor función según diferentes criterios (más
   rápido, más preciso, camino más corto)
5. Proporcionar información detallada por función que permita análisis más profundo
   del comportamiento de cada tipo de potencial

Comportamiento esperado:
    - Buscar automáticamente archivos CSV en directorio logs/
    - Cargar datos de múltiples ejecuciones (cada tipo de potencial)
    - Calcular 8 métricas clave por cada ejecución
    - Generar tabla comparativa formateada en terminal
    - Identificar la función de potencial con mejor desempeño por métrica

Funciones principales:
    
    load_csv(filepath):
        Lee archivo CSV y extrae datos en estructura de diccionario.
        
        Parámetros:
            filepath: Path al archivo CSV
        
        Retorna:
            Lista de dicts con claves: timestamp, x_cm, y_cm, heading_deg,
            v_left_cm_s, v_right_cm_s, v_linear_cm_s, omega_deg_s,
            distance_error_cm, heading_error_deg, potential_type
    
    analyze_trajectory(data):
        Calcula métricas de desempeño a partir de datos de trayectoria.
        
        Parámetros:
            data: Lista de dicts de load_csv()
        
        Retorna:
            Dict con 8 métricas:
                - total_time: Duración total en segundos
                - final_distance_error: Error final en cm
                - avg_distance_error: Error promedio en cm
                - avg_v_linear: Velocidad lineal promedia en cm/s
                - max_v_linear: Velocidad lineal máxima en cm/s
                - avg_omega: Velocidad angular promedio en deg/s
                - path_length: Longitud de trayectoria recorrida en cm
                - samples: Número de muestras registradas
    
    print_comparison(results):
        Imprime tabla formateada con comparación de todas las ejecuciones.
        
        Parámetros:
            results: Dict {tipo_potencial: métricas_analyze_trajectory}
        
        Salida:
            Tabla con columnas por tipo de potencial y filas por métrica,
            destacando el mejor valor de cada métrica.

Métricas calculadas:
    1. Tiempo total: Duración desde inicio hasta convergencia
    2. Error final: Distancia al objetivo al finalizar (precisión)
    3. Error promedio: Error medio durante toda la trayectoria
    4. Velocidad lineal promedio: Eficiencia de movimiento
    5. Velocidad lineal máxima: Agresividad del control
    6. Velocidad angular promedio: Suavidad de giros
    7. Longitud de trayectoria: Integración del camino recorrido
    8. Número de muestras: Cantidad de iteraciones de control

Uso típico:
    python analyze_results.py
    # Analiza automáticamente todos los CSVs en logs/
    # Genera tabla comparativa en terminal

Ejemplo de salida:
    ╔════════════════════════╦═════════╦═══════════╦════════╦═════════════╗
    ║ Métrica                ║ Linear  ║ Quadratic ║ Conic  ║ Exponential ║
    ╠════════════════════════╬═════════╬═══════════╬════════╬═════════════╣
    ║ Tiempo total (s)       ║ 38.08   ║ 35.77*    ║ 36.50  ║ 37.20       ║
    ║ Error final (cm)       ║ 2.70    ║ 2.01*     ║ 2.45   ║ 2.85        ║
    ...
"""

import csv
from pathlib import Path
import statistics


def load_csv(filepath):
    """Carga un CSV de velocidades"""
    data = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({
                'elapsed_s': float(row['elapsed_s']),
                'x_cm': float(row['x_cm']),
                'y_cm': float(row['y_cm']),
                'distance_cm': float(row['distance_cm']),
                'v_left': float(row['v_left']),
                'v_right': float(row['v_right']),
                'v_linear': float(row['v_linear']),
                'potential_type': row['potential_type']
            })
    return data


def analyze_trajectory(data):
    """Analiza estadísticas de una trayectoria"""
    if not data:
        return None
    
    potential_type = data[0]['potential_type']
    total_time = data[-1]['elapsed_s']
    final_distance = data[-1]['distance_cm']
    
    # Calcular distancia recorrida
    total_dist = 0
    for i in range(1, len(data)):
        dx = data[i]['x_cm'] - data[i-1]['x_cm']
        dy = data[i]['y_cm'] - data[i-1]['y_cm']
        total_dist += (dx**2 + dy**2)**0.5
    
    # Velocidades
    v_lefts = [abs(d['v_left']) for d in data]
    v_rights = [abs(d['v_right']) for d in data]
    v_linears = [abs(d['v_linear']) for d in data]
    
    return {
        'potential_type': potential_type,
        'total_time': total_time,
        'final_distance': final_distance,
        'path_length': total_dist,
        'avg_v_left': statistics.mean(v_lefts),
        'max_v_left': max(v_lefts),
        'avg_v_right': statistics.mean(v_rights),
        'max_v_right': max(v_rights),
        'avg_v_linear': statistics.mean(v_linears),
        'max_v_linear': max(v_linears),
        'num_samples': len(data)
    }


def main():
    """Analiza todos los CSV en la carpeta logs"""
    
    # Obtener ruta relativa al directorio raíz del proyecto
    project_root = Path(__file__).parent.parent
    logs_dir = project_root / "logs"
    if not logs_dir.exists():
        print("❌ No existe la carpeta 'logs'")
        return
    
    csv_files = list(logs_dir.glob("velocities_*.csv"))
    
    if not csv_files:
        print("❌ No se encontraron archivos CSV de velocidades")
        print("   Ejecuta primero: python PRM01_P01.py --potential <tipo>")
        return
    
    print("\n" + "="*80)
    print("📊 ANÁLISIS COMPARATIVO DE FUNCIONES DE POTENCIAL")
    print("="*80)
    
    results = []
    
    for csv_file in sorted(csv_files):
        print(f"\nAnalizando: {csv_file.name}")
        data = load_csv(csv_file)
        stats = analyze_trajectory(data)
        if stats:
            results.append(stats)
    
    if not results:
        print("\n❌ No se pudo analizar ningún archivo")
        return
    
    # Tabla comparativa
    print("\n" + "="*80)
    print("📈 COMPARACIÓN DE RESULTADOS")
    print("="*80)
    print(f"{'Potencial':<15} {'Tiempo(s)':<12} {'Error(cm)':<12} {'Dist.Rec(cm)':<15} {'V_media(cm/s)':<15}")
    print("-"*80)
    
    for r in results:
        print(f"{r['potential_type']:<15} "
              f"{r['total_time']:>10.2f}  "
              f"{r['final_distance']:>10.2f}  "
              f"{r['path_length']:>13.2f}  "
              f"{r['avg_v_linear']:>13.2f}")
    
    print("="*80)
    
    # Detalles individuales
    print("\n" + "="*80)
    print("📋 DETALLES POR FUNCIÓN")
    print("="*80)
    
    for r in results:
        print(f"\n🔹 {r['potential_type'].upper()}")
        print(f"   Tiempo total: {r['total_time']:.2f} s")
        print(f"   Error final: {r['final_distance']:.2f} cm")
        print(f"   Distancia recorrida: {r['path_length']:.2f} cm")
        print(f"   Velocidad lineal: media={r['avg_v_linear']:.2f} cm/s, max={r['max_v_linear']:.2f} cm/s")
        print(f"   Velocidad rueda izq: media={r['avg_v_left']:.2f} cm/s, max={r['max_v_left']:.2f} cm/s")
        print(f"   Velocidad rueda der: media={r['avg_v_right']:.2f} cm/s, max={r['max_v_right']:.2f} cm/s")
        print(f"   Muestras: {r['num_samples']}")
    
    # Mejor función
    print("\n" + "="*80)
    print("🏆 MEJOR FUNCIÓN POR CRITERIO")
    print("="*80)
    
    fastest = min(results, key=lambda x: x['total_time'])
    print(f"⏱️  Más rápido: {fastest['potential_type']} ({fastest['total_time']:.2f} s)")
    
    most_accurate = min(results, key=lambda x: x['final_distance'])
    print(f"🎯 Más preciso: {most_accurate['potential_type']} (error {most_accurate['final_distance']:.2f} cm)")
    
    shortest_path = min(results, key=lambda x: x['path_length'])
    print(f"📏 Camino más corto: {shortest_path['potential_type']} ({shortest_path['path_length']:.2f} cm)")
    
    print("="*80)


if __name__ == "__main__":
    main()
