#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analizador de Logs CSV de Navegación del Robot Create 3

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
    11 de noviembre de 2025

===============================================================================
DESCRIPCIÓN
===============================================================================

Este script permite analizar los archivos CSV generados durante la navegación
del robot, proporcionando métricas detalladas, visualizaciones y comparaciones
entre diferentes ejecuciones.

Características principales:
- Análisis de un archivo CSV individual
- Comparación entre múltiples archivos CSV
- Cálculo de métricas de desempeño (tiempo, distancia, velocidad, precisión)
- Visualización de trayectorias en 2D
- Gráficos de velocidad, distancia al objetivo, y errores
- Exportación de resúmenes en texto y CSV

Uso:
    python utils/analyze_logs.py logs/velocities_conic_combined_20251113_171126.csv
    python utils/analyze_logs.py logs/ --compare
    python utils/analyze_logs.py logs/ --plot
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import matplotlib.pyplot as plt
    import numpy as np
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("[WARNING] matplotlib no disponible. Las visualizaciones estarán deshabilitadas.")


# ══════════════════════════════════════════════════════════
#  FUNCIONES DE CARGA DE DATOS
# ══════════════════════════════════════════════════════════

def load_map_data(json_path: Optional[Path] = None) -> Optional[Dict]:
    """
    Carga los datos del mapa desde un archivo JSON.
    
    Args:
        json_path: Ruta al archivo JSON. Si es None, busca en data/points.json
        
    Returns:
        Diccionario con q_i, waypoints, q_f o None si no se encuentra
    """
    if json_path is None:
        # Buscar en ubicaciones comunes
        possible_paths = [
            Path('data/points.json'),
            Path('PL5/data/points.json'),
            Path('../data/points.json'),
        ]
        for path in possible_paths:
            if path.exists():
                json_path = path
                break
        
        if json_path is None:
            return None
    
    json_path = Path(json_path)
    
    if not json_path.exists():
        return None
    
    try:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return data
    except (json.JSONDecodeError, IOError) as e:
        print(f"[WARNING] No se pudo cargar el mapa desde {json_path}: {e}")
        return None


# ══════════════════════════════════════════════════════════
#  FUNCIONES DE ANÁLISIS
# ══════════════════════════════════════════════════════════

def load_csv(filepath: Path) -> Tuple[List[Dict], Dict]:
    """
    Carga un archivo CSV y retorna los datos y metadatos.
    
    Args:
        filepath: Ruta al archivo CSV
        
    Returns:
        Tupla con (datos, metadatos) donde:
            - datos: Lista de diccionarios con los datos de cada fila
            - metadatos: Diccionario con información del archivo
    """
    if not filepath.exists():
        print(f"[ERROR] Archivo no encontrado: {filepath}")
        sys.exit(1)
    
    data = []
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Convertir valores numéricos
            try:
                row['elapsed_s'] = float(row['elapsed_s'])
                row['x_cm'] = float(row['x_cm'])
                row['y_cm'] = float(row['y_cm'])
                row['theta_deg'] = float(row['theta_deg'])
                row['distance_cm'] = float(row['distance_cm'])
                row['v_left'] = float(row['v_left'])
                row['v_right'] = float(row['v_right'])
                row['v_linear'] = float(row['v_linear'])
                row['omega'] = float(row['omega'])
                row['angle_error_deg'] = float(row['angle_error_deg'])
                row['fx_repulsive'] = float(row.get('fx_repulsive', 0))
                row['fy_repulsive'] = float(row.get('fy_repulsive', 0))
                row['num_obstacles'] = int(row.get('num_obstacles', 0))
            except (ValueError, KeyError) as e:
                print(f"[WARNING] Error al parsear fila: {e}")
                continue
            
            data.append(row)
    
    metadata = {
        'filename': filepath.name,
        'filepath': str(filepath),
        'num_rows': len(data),
        'potential_type': data[0].get('potential_type', 'unknown') if data else 'unknown'
    }
    
    return data, metadata


def calculate_metrics(data: List[Dict]) -> Dict:
    """
    Calcula métricas de desempeño a partir de los datos.
    
    Args:
        data: Lista de diccionarios con datos de navegación
        
    Returns:
        Diccionario con métricas calculadas
    """
    if not data:
        return {}
    
    # Tiempo total
    total_time = data[-1]['elapsed_s'] - data[0]['elapsed_s']
    
    # Distancia inicial y final
    initial_distance = data[0]['distance_cm']
    final_distance = data[-1]['distance_cm']
    
    # Distancia recorrida (suma de desplazamientos)
    total_distance = 0.0
    for i in range(1, len(data)):
        dx = data[i]['x_cm'] - data[i-1]['x_cm']
        dy = data[i]['y_cm'] - data[i-1]['y_cm']
        total_distance += math.hypot(dx, dy)
    
    # Eficiencia de trayectoria (distancia directa / distancia recorrida)
    if total_distance > 0:
        efficiency = initial_distance / total_distance
    else:
        efficiency = 0.0
    
    # Velocidades
    velocities_linear = [abs(row['v_linear']) for row in data]
    velocities_left = [abs(row['v_left']) for row in data]
    velocities_right = [abs(row['v_right']) for row in data]
    
    avg_velocity = sum(velocities_linear) / len(velocities_linear) if velocities_linear else 0.0
    max_velocity = max(velocities_linear) if velocities_linear else 0.0
    
    # Velocidad angular
    omegas = [abs(row['omega']) for row in data]
    avg_omega = sum(omegas) / len(omegas) if omegas else 0.0
    max_omega = max(omegas) if omegas else 0.0
    
    # Errores
    distances = [row['distance_cm'] for row in data]
    angle_errors = [abs(row['angle_error_deg']) for row in data]
    
    avg_distance_error = sum(distances) / len(distances) if distances else 0.0
    avg_angle_error = sum(angle_errors) / len(angle_errors) if angle_errors else 0.0
    
    # Obstáculos detectados (si aplica)
    obstacles_detected = sum(1 for row in data if row.get('num_obstacles', 0) > 0)
    obstacles_percentage = (obstacles_detected / len(data)) * 100 if data else 0.0
    
    # Fuerzas repulsivas promedio
    fx_rep = [abs(row.get('fx_repulsive', 0)) for row in data]
    fy_rep = [abs(row.get('fy_repulsive', 0)) for row in data]
    avg_fx_rep = sum(fx_rep) / len(fx_rep) if fx_rep else 0.0
    avg_fy_rep = sum(fy_rep) / len(fy_rep) if fy_rep else 0.0
    
    metrics = {
        'total_time_s': total_time,
        'total_time_min': total_time / 60.0,
        'initial_distance_cm': initial_distance,
        'final_distance_cm': final_distance,
        'distance_reduction_cm': initial_distance - final_distance,
        'total_distance_traveled_cm': total_distance,
        'efficiency': efficiency,
        'avg_velocity_cm_s': avg_velocity,
        'max_velocity_cm_s': max_velocity,
        'avg_omega_deg_s': avg_omega,
        'max_omega_deg_s': max_omega,
        'avg_distance_error_cm': avg_distance_error,
        'avg_angle_error_deg': avg_angle_error,
        'final_error_cm': final_distance,
        'obstacles_detected_count': obstacles_detected,
        'obstacles_percentage': obstacles_percentage,
        'avg_fx_repulsive': avg_fx_rep,
        'avg_fy_repulsive': avg_fy_rep,
        'num_samples': len(data)
    }
    
    return metrics


def print_metrics(metadata: Dict, metrics: Dict):
    """
    Imprime las métricas de forma formateada.
    
    Args:
        metadata: Metadatos del archivo
        metrics: Métricas calculadas
    """
    print("\n" + "="*70)
    print(f"ANÁLISIS DE LOG: {metadata['filename']}")
    print("="*70)
    print(f"Tipo de potencial: {metadata['potential_type']}")
    print(f"Número de muestras: {metadata['num_rows']}")
    
    print("\nMÉTRICAS DE DESEMPEÑO:")
    print("-" * 70)
    print(f"Tiempo total:           {metrics['total_time_s']:.2f} s ({metrics['total_time_min']:.2f} min)")
    print(f"Distancia inicial:       {metrics['initial_distance_cm']:.2f} cm")
    print(f"Distancia final:         {metrics['final_distance_cm']:.2f} cm")
    print(f"Reducción de distancia:  {metrics['distance_reduction_cm']:.2f} cm")
    print(f"Distancia recorrida:    {metrics['total_distance_traveled_cm']:.2f} cm")
    print(f"Eficiencia de trayectoria: {metrics['efficiency']:.3f} ({metrics['efficiency']*100:.1f}%)")
    
    print("\nVELOCIDADES:")
    print("-" * 70)
    print(f"Velocidad promedio:      {metrics['avg_velocity_cm_s']:.2f} cm/s")
    print(f"Velocidad máxima:        {metrics['max_velocity_cm_s']:.2f} cm/s")
    print(f"Velocidad angular prom:  {metrics['avg_omega_deg_s']:.2f} deg/s")
    print(f"Velocidad angular máx:   {metrics['max_omega_deg_s']:.2f} deg/s")
    
    print("\nERRORES:")
    print("-" * 70)
    print(f"Error distancia promedio: {metrics['avg_distance_error_cm']:.2f} cm")
    print(f"Error angular promedio:   {metrics['avg_angle_error_deg']:.2f} deg")
    print(f"Error final:              {metrics['final_error_cm']:.2f} cm")
    
    if metrics['obstacles_detected_count'] > 0:
        print("\nOBSTÁCULOS:")
        print("-" * 70)
        print(f"Iteraciones con obstáculos: {metrics['obstacles_detected_count']} ({metrics['obstacles_percentage']:.1f}%)")
        print(f"Fuerza repulsiva X promedio: {metrics['avg_fx_repulsive']:.2f}")
        print(f"Fuerza repulsiva Y promedio: {metrics['avg_fy_repulsive']:.2f}")
    
    print("="*70 + "\n")


def plot_trajectory(data: List[Dict], metadata: Dict, save_path: Optional[Path] = None):
    """
    Genera un gráfico avanzado de la trayectoria del robot con múltiples visualizaciones.
    
    Args:
        data: Datos de navegación
        metadata: Metadatos del archivo
        save_path: Ruta opcional para guardar el gráfico
    """
    if not HAS_MATPLOTLIB:
        print("[WARNING] matplotlib no disponible. No se puede generar gráfico.")
        return
    
    x_coords = [row['x_cm'] for row in data]
    y_coords = [row['y_cm'] for row in data]
    velocities = [abs(row['v_linear']) for row in data]
    distances = [row['distance_cm'] for row in data]
    times = [row['elapsed_s'] for row in data]
    obstacles = [row.get('num_obstacles', 0) for row in data]
    fx_rep = [row.get('fx_repulsive', 0) for row in data]
    fy_rep = [row.get('fy_repulsive', 0) for row in data]
    
    # Crear figura con subplots
    fig = plt.figure(figsize=(20, 12))
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 1: Trayectoria con colores según velocidad
    # ═══════════════════════════════════════════════════════════
    ax1 = fig.add_subplot(gs[0, 0])
    scatter = ax1.scatter(x_coords, y_coords, c=velocities, cmap='viridis', 
                         s=30, alpha=0.7, edgecolors='black', linewidths=0.5)
    ax1.plot(x_coords, y_coords, 'k-', linewidth=1, alpha=0.3, zorder=0)
    ax1.scatter(x_coords[0], y_coords[0], c='green', s=300, marker='o', 
                label='Inicio', zorder=10, edgecolors='black', linewidths=3)
    ax1.scatter(x_coords[-1], y_coords[-1], c='red', s=300, marker='s', 
                label='Final', zorder=10, edgecolors='black', linewidths=3)
    
    # Flechas de dirección cada N puntos
    step = max(1, len(data) // 20)
    for i in range(0, len(data)-1, step):
        if i + 1 < len(data):
            dx = x_coords[i+1] - x_coords[i]
            dy = y_coords[i+1] - y_coords[i]
            if abs(dx) > 0.1 or abs(dy) > 0.1:
                ax1.arrow(x_coords[i], y_coords[i], dx*0.3, dy*0.3,
                         head_width=3, head_length=2, fc='black', ec='black', 
                         alpha=0.5, zorder=5, length_includes_head=True)
    
    plt.colorbar(scatter, ax=ax1, label='Velocidad (cm/s)')
    ax1.set_xlabel('X (cm)', fontsize=11)
    ax1.set_ylabel('Y (cm)', fontsize=11)
    ax1.set_title('Trayectoria Coloreada por Velocidad', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=9)
    ax1.set_aspect('equal', adjustable='box')
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 2: Trayectoria con colores según tiempo
    # ═══════════════════════════════════════════════════════════
    ax2 = fig.add_subplot(gs[0, 1])
    scatter2 = ax2.scatter(x_coords, y_coords, c=times, cmap='plasma', 
                           s=30, alpha=0.7, edgecolors='black', linewidths=0.5)
    ax2.plot(x_coords, y_coords, 'k-', linewidth=1, alpha=0.3, zorder=0)
    ax2.scatter(x_coords[0], y_coords[0], c='green', s=300, marker='o', 
                label='Inicio', zorder=10, edgecolors='black', linewidths=3)
    ax2.scatter(x_coords[-1], y_coords[-1], c='red', s=300, marker='s', 
                label='Final', zorder=10, edgecolors='black', linewidths=3)
    plt.colorbar(scatter2, ax=ax2, label='Tiempo (s)')
    ax2.set_xlabel('X (cm)', fontsize=11)
    ax2.set_ylabel('Y (cm)', fontsize=11)
    ax2.set_title('Trayectoria Coloreada por Tiempo', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=9)
    ax2.set_aspect('equal', adjustable='box')
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 3: Trayectoria con vectores de fuerza repulsiva
    # ═══════════════════════════════════════════════════════════
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(x_coords, y_coords, 'b-', linewidth=2, alpha=0.5, label='Trayectoria', zorder=1)
    ax3.scatter(x_coords[0], y_coords[0], c='green', s=300, marker='o', 
                label='Inicio', zorder=10, edgecolors='black', linewidths=3)
    ax3.scatter(x_coords[-1], y_coords[-1], c='red', s=300, marker='s', 
                label='Final', zorder=10, edgecolors='black', linewidths=3)
    
    # Dibujar vectores de fuerza repulsiva donde hay obstáculos
    step_force = max(1, len(data) // 30)
    max_force = max([math.hypot(fx, fy) for fx, fy in zip(fx_rep, fy_rep)])
    if max_force > 0:
        for i in range(0, len(data), step_force):
            if obstacles[i] > 0:
                fx, fy = fx_rep[i], fy_rep[i]
                force_mag = math.hypot(fx, fy)
                if force_mag > 0.01:  # Solo dibujar si hay fuerza significativa
                    scale = 20 / max_force  # Escalar para visualización
                    ax3.arrow(x_coords[i], y_coords[i], fx*scale, fy*scale,
                             head_width=2, head_length=1.5, fc='red', ec='red',
                             alpha=0.6, zorder=5, length_includes_head=True)
    
    ax3.set_xlabel('X (cm)', fontsize=11)
    ax3.set_ylabel('Y (cm)', fontsize=11)
    ax3.set_title('Trayectoria con Fuerzas Repulsivas', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=9)
    ax3.set_aspect('equal', adjustable='box')
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 4: Distancia al objetivo vs Tiempo
    # ═══════════════════════════════════════════════════════════
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(times, distances, 'r-', linewidth=2.5, label='Distancia al objetivo')
    ax4.fill_between(times, distances, alpha=0.3, color='red')
    ax4.axhline(y=distances[-1], color='green', linestyle='--', linewidth=2, 
                label=f'Distancia final: {distances[-1]:.2f} cm')
    ax4.set_xlabel('Tiempo (s)', fontsize=11)
    ax4.set_ylabel('Distancia al objetivo (cm)', fontsize=11)
    ax4.set_title('Evolución de la Distancia al Objetivo', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 5: Velocidades (lineal y angular)
    # ═══════════════════════════════════════════════════════════
    ax5 = fig.add_subplot(gs[1, 1])
    omegas = [abs(row['omega']) for row in data]
    ax5_twin = ax5.twinx()
    
    line1 = ax5.plot(times, velocities, 'b-', linewidth=2.5, label='Velocidad lineal (cm/s)')
    line2 = ax5_twin.plot(times, omegas, 'g-', linewidth=2.5, label='Velocidad angular (deg/s)')
    
    ax5.set_xlabel('Tiempo (s)', fontsize=11)
    ax5.set_ylabel('Velocidad Lineal (cm/s)', fontsize=11, color='b')
    ax5_twin.set_ylabel('Velocidad Angular (deg/s)', fontsize=11, color='g')
    ax5.set_title('Velocidades del Robot', fontsize=12, fontweight='bold')
    ax5.tick_params(axis='y', labelcolor='b')
    ax5_twin.tick_params(axis='y', labelcolor='g')
    ax5.grid(True, alpha=0.3)
    
    # Leyenda combinada
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax5.legend(lines, labels, loc='upper right', fontsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 6: Velocidades de ruedas
    # ═══════════════════════════════════════════════════════════
    ax6 = fig.add_subplot(gs[1, 2])
    v_left = [abs(row['v_left']) for row in data]
    v_right = [abs(row['v_right']) for row in data]
    
    ax6.plot(times, v_left, 'b-', linewidth=2, label='Rueda izquierda', alpha=0.8)
    ax6.plot(times, v_right, 'r-', linewidth=2, label='Rueda derecha', alpha=0.8)
    ax6.fill_between(times, v_left, alpha=0.2, color='blue')
    ax6.fill_between(times, v_right, alpha=0.2, color='red')
    
    ax6.set_xlabel('Tiempo (s)', fontsize=11)
    ax6.set_ylabel('Velocidad (cm/s)', fontsize=11)
    ax6.set_title('Velocidades de las Ruedas', fontsize=12, fontweight='bold')
    ax6.grid(True, alpha=0.3)
    ax6.legend(fontsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 7: Error angular vs Tiempo
    # ═══════════════════════════════════════════════════════════
    ax7 = fig.add_subplot(gs[2, 0])
    angle_errors = [abs(row['angle_error_deg']) for row in data]
    ax7.plot(times, angle_errors, 'g-', linewidth=2.5, label='Error angular')
    ax7.fill_between(times, angle_errors, alpha=0.3, color='green')
    ax7.axhline(y=angle_errors[-1], color='orange', linestyle='--', linewidth=2,
                label=f'Error final: {angle_errors[-1]:.2f}°')
    ax7.set_xlabel('Tiempo (s)', fontsize=11)
    ax7.set_ylabel('Error Angular (deg)', fontsize=11)
    ax7.set_title('Error Angular vs Tiempo', fontsize=12, fontweight='bold')
    ax7.grid(True, alpha=0.3)
    ax7.legend(fontsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 8: Obstáculos detectados
    # ═══════════════════════════════════════════════════════════
    ax8 = fig.add_subplot(gs[2, 1])
    ax8.fill_between(times, obstacles, alpha=0.5, color='orange', step='pre')
    ax8.plot(times, obstacles, 'o-', linewidth=2, markersize=4, color='orange', label='Obstáculos')
    ax8.set_xlabel('Tiempo (s)', fontsize=11)
    ax8.set_ylabel('Número de Obstáculos', fontsize=11)
    ax8.set_title('Obstáculos Detectados en el Tiempo', fontsize=12, fontweight='bold')
    ax8.grid(True, alpha=0.3)
    ax8.legend(fontsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 9: Magnitud de fuerzas repulsivas
    # ═══════════════════════════════════════════════════════════
    ax9 = fig.add_subplot(gs[2, 2])
    force_magnitudes = [math.hypot(fx, fy) for fx, fy in zip(fx_rep, fy_rep)]
    ax9.plot(times, force_magnitudes, 'purple', linewidth=2.5, label='Magnitud fuerza repulsiva')
    ax9.fill_between(times, force_magnitudes, alpha=0.3, color='purple')
    ax9.set_xlabel('Tiempo (s)', fontsize=11)
    ax9.set_ylabel('Magnitud Fuerza Repulsiva', fontsize=11)
    ax9.set_title('Fuerzas Repulsivas vs Tiempo', fontsize=12, fontweight='bold')
    ax9.grid(True, alpha=0.3)
    ax9.legend(fontsize=9)
    
    # Título general
    fig.suptitle(
        f'ANÁLISIS COMPLETO DE NAVEGACIÓN - {metadata["filename"]}\n'
        f'Tipo de Potencial: {metadata["potential_type"].upper()} | '
        f'Muestras: {len(data)} | '
        f'Tiempo total: {times[-1]:.2f}s | '
        f'Distancia final: {distances[-1]:.2f} cm',
        fontsize=16, fontweight='bold', y=0.995
    )
    
    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight', facecolor='white')
        print(f"Gráfico completo guardado: {save_path}")
    else:
        plt.show()


def plot_metrics(data: List[Dict], metadata: Dict, save_path: Optional[Path] = None):
    """
    Genera gráficos avanzados de métricas temporales con análisis detallado.
    
    Args:
        data: Datos de navegación
        metadata: Metadatos del archivo
        save_path: Ruta opcional para guardar el gráfico
    """
    if not HAS_MATPLOTLIB:
        print("[WARNING] matplotlib no disponible. No se puede generar gráfico.")
        return
    
    times = [row['elapsed_s'] for row in data]
    distances = [row['distance_cm'] for row in data]
    velocities = [abs(row['v_linear']) for row in data]
    angle_errors = [abs(row['angle_error_deg']) for row in data]
    omegas = [abs(row['omega']) for row in data]
    v_left = [abs(row['v_left']) for row in data]
    v_right = [abs(row['v_right']) for row in data]
    obstacles = [row.get('num_obstacles', 0) for row in data]
    fx_rep = [row.get('fx_repulsive', 0) for row in data]
    fy_rep = [row.get('fy_repulsive', 0) for row in data]
    force_mags = [math.hypot(fx, fy) for fx, fy in zip(fx_rep, fy_rep)]
    
    # Crear figura con múltiples subplots
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 1: Distancia al objetivo con zonas de mejora
    # ═══════════════════════════════════════════════════════════
    ax1 = axes[0, 0]
    ax1.plot(times, distances, 'r-', linewidth=2.5, label='Distancia al objetivo')
    ax1.fill_between(times, distances, alpha=0.3, color='red')
    
    # Calcular velocidad de aproximación (derivada numérica)
    if len(distances) > 1:
        approach_rates = []
        for i in range(1, len(distances)):
            dt = times[i] - times[i-1]
            if dt > 0:
                rate = (distances[i-1] - distances[i]) / dt  # cm/s de reducción
                approach_rates.append(rate)
            else:
                approach_rates.append(0)
        approach_rates.insert(0, approach_rates[0] if approach_rates else 0)
        
        # Marcar zonas de alta velocidad de aproximación
        avg_rate = sum(approach_rates) / len(approach_rates) if approach_rates else 0
        for i, rate in enumerate(approach_rates):
            if rate > avg_rate * 1.5 and i < len(times):
                ax1.axvspan(times[i], times[min(i+1, len(times)-1)], 
                           alpha=0.2, color='green', zorder=0)
    
    ax1.axhline(y=distances[-1], color='green', linestyle='--', linewidth=2,
                label=f'Distancia final: {distances[-1]:.2f} cm')
    ax1.set_xlabel('Tiempo (s)', fontsize=11)
    ax1.set_ylabel('Distancia al objetivo (cm)', fontsize=11)
    ax1.set_title('Evolución de la Distancia al Objetivo', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 2: Velocidad con estadísticas
    # ═══════════════════════════════════════════════════════════
    ax2 = axes[0, 1]
    ax2.plot(times, velocities, 'b-', linewidth=2.5, label='Velocidad lineal')
    ax2.fill_between(times, velocities, alpha=0.3, color='blue')
    
    # Líneas de estadísticas
    avg_vel = sum(velocities) / len(velocities) if velocities else 0
    max_vel = max(velocities) if velocities else 0
    ax2.axhline(y=avg_vel, color='orange', linestyle='--', linewidth=2,
                label=f'Promedio: {avg_vel:.2f} cm/s')
    ax2.axhline(y=max_vel, color='red', linestyle='--', linewidth=2,
                label=f'Máxima: {max_vel:.2f} cm/s')
    
    ax2.set_xlabel('Tiempo (s)', fontsize=11)
    ax2.set_ylabel('Velocidad lineal (cm/s)', fontsize=11)
    ax2.set_title('Velocidad Lineal vs Tiempo', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 3: Error angular con correcciones
    # ═══════════════════════════════════════════════════════════
    ax3 = axes[1, 0]
    ax3.plot(times, angle_errors, 'g-', linewidth=2.5, label='Error angular')
    ax3.fill_between(times, angle_errors, alpha=0.3, color='green')
    
    # Marcar correcciones importantes (reducciones bruscas)
    for i in range(1, len(angle_errors)):
        if angle_errors[i] < angle_errors[i-1] - 5:  # Reducción de más de 5 grados
            ax3.scatter(times[i], angle_errors[i], c='red', s=50, 
                       marker='v', zorder=5, alpha=0.7)
    
    ax3.axhline(y=angle_errors[-1], color='orange', linestyle='--', linewidth=2,
                label=f'Error final: {angle_errors[-1]:.2f}°')
    ax3.set_xlabel('Tiempo (s)', fontsize=11)
    ax3.set_ylabel('Error angular (deg)', fontsize=11)
    ax3.set_title('Error Angular vs Tiempo (▼ = Correcciones)', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 4: Velocidad angular
    # ═══════════════════════════════════════════════════════════
    ax4 = axes[1, 1]
    ax4.plot(times, omegas, 'purple', linewidth=2.5, label='Velocidad angular')
    ax4.fill_between(times, omegas, alpha=0.3, color='purple')
    
    avg_omega = sum(omegas) / len(omegas) if omegas else 0
    max_omega = max(omegas) if omegas else 0
    ax4.axhline(y=avg_omega, color='orange', linestyle='--', linewidth=2,
                label=f'Promedio: {avg_omega:.2f} deg/s')
    ax4.axhline(y=max_omega, color='red', linestyle='--', linewidth=2,
                label=f'Máxima: {max_omega:.2f} deg/s')
    
    ax4.set_xlabel('Tiempo (s)', fontsize=11)
    ax4.set_ylabel('Velocidad angular (deg/s)', fontsize=11)
    ax4.set_title('Velocidad Angular vs Tiempo', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 5: Diferencia de velocidades de ruedas (control)
    # ═══════════════════════════════════════════════════════════
    ax5 = axes[2, 0]
    wheel_diff = [abs(v_left[i] - v_right[i]) for i in range(len(v_left))]
    ax5.plot(times, wheel_diff, 'brown', linewidth=2.5, label='Diferencia |v_left - v_right|')
    ax5.fill_between(times, wheel_diff, alpha=0.3, color='brown')
    
    # Mostrar también las velocidades individuales
    ax5.plot(times, v_left, 'b--', linewidth=1.5, alpha=0.6, label='Rueda izquierda')
    ax5.plot(times, v_right, 'r--', linewidth=1.5, alpha=0.6, label='Rueda derecha')
    
    ax5.set_xlabel('Tiempo (s)', fontsize=11)
    ax5.set_ylabel('Velocidad (cm/s)', fontsize=11)
    ax5.set_title('Control de Velocidades de Ruedas', fontsize=12, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    ax5.legend(fontsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 6: Fuerzas repulsivas y obstáculos
    # ═══════════════════════════════════════════════════════════
    ax6 = axes[2, 1]
    ax6_twin = ax6.twinx()
    
    # Fuerza repulsiva
    line1 = ax6.plot(times, force_mags, 'purple', linewidth=2.5, label='Magnitud fuerza repulsiva')
    ax6.fill_between(times, force_mags, alpha=0.3, color='purple')
    
    # Obstáculos
    line2 = ax6_twin.plot(times, obstacles, 'orange', linewidth=2, 
                         marker='o', markersize=4, label='Número de obstáculos')
    
    ax6.set_xlabel('Tiempo (s)', fontsize=11)
    ax6.set_ylabel('Magnitud Fuerza Repulsiva', fontsize=11, color='purple')
    ax6_twin.set_ylabel('Número de Obstáculos', fontsize=11, color='orange')
    ax6.set_title('Fuerzas Repulsivas y Obstáculos Detectados', fontsize=12, fontweight='bold')
    ax6.tick_params(axis='y', labelcolor='purple')
    ax6_twin.tick_params(axis='y', labelcolor='orange')
    ax6.grid(True, alpha=0.3)
    
    # Leyenda combinada
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax6.legend(lines, labels, loc='upper right', fontsize=9)
    
    plt.suptitle(
        f'MÉTRICAS DETALLADAS DE NAVEGACIÓN - {metadata["filename"]}\n'
        f'Potencial: {metadata["potential_type"].upper()} | '
        f'Tiempo total: {times[-1]:.2f}s | '
        f'Distancia final: {distances[-1]:.2f} cm | '
        f'Velocidad promedio: {avg_vel:.2f} cm/s',
        fontsize=14, fontweight='bold', y=0.995
    )
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight', facecolor='white')
        print(f"Gráfico de métricas guardado: {save_path}")
    else:
        plt.show()


def plot_map_with_trajectory(data: List[Dict], metadata: Dict, 
                             map_data: Optional[Dict] = None,
                             save_path: Optional[Path] = None):
    """
    Genera un mapa completo con el entorno planificado y la trayectoria real del robot.
    
    Args:
        data: Datos de navegación del robot
        metadata: Metadatos del archivo CSV
        map_data: Datos del mapa desde points.json (q_i, waypoints, q_f)
        save_path: Ruta opcional para guardar el gráfico
    """
    if not HAS_MATPLOTLIB:
        print("[WARNING] matplotlib no disponible. No se puede generar mapa.")
        return
    
    # Extraer trayectoria del robot
    x_coords = [row['x_cm'] for row in data]
    y_coords = [row['y_cm'] for row in data]
    velocities = [abs(row['v_linear']) for row in data]
    times = [row['elapsed_s'] for row in data]
    
    # Crear figura
    fig, ax = plt.subplots(figsize=(16, 14))
    
    # ═══════════════════════════════════════════════════════════
    # DIBUJAR MAPA BASE (si está disponible)
    # ═══════════════════════════════════════════════════════════
    if map_data:
        # Dibujar cuadrícula del mapa (500x500 cm según documentación)
        map_size = 1000  # Ajustar según el tamaño real del mapa
        ax.set_xlim(-50, map_size + 50)
        ax.set_ylim(-50, map_size + 50)
        ax.grid(True, alpha=0.2, linestyle='--', linewidth=0.5)
        
        # Punto inicial planificado (q_i)
        if 'q_i' in map_data:
            q_i = map_data['q_i']
            qi_x, qi_y = q_i['x'], q_i['y']
            qi_theta = q_i.get('theta', 0)
            
            # Dibujar punto inicial planificado
            ax.scatter(qi_x, qi_y, c='lime', s=400, marker='o', 
                      edgecolors='darkgreen', linewidths=3, 
                      label='Inicio Planificado (q_i)', zorder=10, alpha=0.8)
            
            # Flecha de orientación inicial planificada
            arrow_length = 30
            qi_dx = arrow_length * math.cos(math.radians(qi_theta))
            qi_dy = arrow_length * math.sin(math.radians(qi_theta))
            ax.arrow(qi_x, qi_y, qi_dx, qi_dy, 
                    head_width=15, head_length=10, fc='darkgreen', ec='darkgreen',
                    linewidth=2, zorder=11, alpha=0.7)
        
        # Waypoints planificados
        if 'waypoints' in map_data and map_data['waypoints']:
            waypoints = map_data['waypoints']
            wp_x = [wp['x'] for wp in waypoints]
            wp_y = [wp['y'] for wp in waypoints]
            
            # Dibujar waypoints planificados
            ax.scatter(wp_x, wp_y, c='gold', s=300, marker='*', 
                      edgecolors='orange', linewidths=2,
                      label=f'Waypoints Planificados ({len(waypoints)})', 
                      zorder=9, alpha=0.8)
            
            # Conectar waypoints con líneas punteadas
            if len(waypoints) > 1:
                for i in range(len(waypoints) - 1):
                    ax.plot([wp_x[i], wp_x[i+1]], [wp_y[i], wp_y[i+1]], 
                           'k--', linewidth=1.5, alpha=0.4, zorder=1)
            
            # Conectar q_i al primer waypoint
            if 'q_i' in map_data:
                ax.plot([qi_x, wp_x[0]], [qi_y, wp_y[0]], 
                       'k--', linewidth=1.5, alpha=0.4, zorder=1)
        
        # Punto final planificado (q_f)
        if 'q_f' in map_data:
            q_f = map_data['q_f']
            qf_x, qf_y = q_f['x'], q_f['y']
            
            ax.scatter(qf_x, qf_y, c='red', s=400, marker='s', 
                      edgecolors='darkred', linewidths=3,
                      label='Final Planificado (q_f)', zorder=10, alpha=0.8)
            
            # Conectar último waypoint a q_f
            if 'waypoints' in map_data and map_data['waypoints']:
                last_wp = waypoints[-1]
                ax.plot([last_wp['x'], qf_x], [last_wp['y'], qf_y], 
                       'k--', linewidth=1.5, alpha=0.4, zorder=1)
            elif 'q_i' in map_data:
                # Si no hay waypoints, conectar q_i a q_f
                ax.plot([qi_x, qf_x], [qi_y, qf_y], 
                       'k--', linewidth=1.5, alpha=0.4, zorder=1, 
                       label='Ruta Planificada')
        
        # Anotar números en waypoints
        if 'waypoints' in map_data and map_data['waypoints']:
            for i, wp in enumerate(waypoints, 1):
                ax.annotate(f'WP{i}', (wp['x'], wp['y']), 
                           fontsize=9, fontweight='bold', 
                           xytext=(5, 5), textcoords='offset points',
                           bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
    else:
        # Si no hay mapa, ajustar límites según trayectoria
        margin = 50
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        ax.set_xlim(x_min - margin, x_max + margin)
        ax.set_ylim(y_min - margin, y_max + margin)
        ax.grid(True, alpha=0.2)
    
    # ═══════════════════════════════════════════════════════════
    # DIBUJAR TRAYECTORIA REAL DEL ROBOT
    # ═══════════════════════════════════════════════════════════
    
    # Trayectoria coloreada por velocidad
    scatter = ax.scatter(x_coords, y_coords, c=velocities, cmap='viridis', 
                        s=40, alpha=0.7, edgecolors='black', linewidths=0.5,
                        zorder=5, label='Trayectoria Real')
    
    # Línea de trayectoria
    ax.plot(x_coords, y_coords, 'k-', linewidth=1.5, alpha=0.3, zorder=4)
    
    # Punto inicial real
    ax.scatter(x_coords[0], y_coords[0], c='green', s=350, marker='o', 
              edgecolors='darkgreen', linewidths=3,
              label='Inicio Real', zorder=12, alpha=0.9)
    
    # Flecha de dirección inicial real
    if len(data) > 1:
        dx = x_coords[1] - x_coords[0]
        dy = y_coords[1] - y_coords[0]
        if abs(dx) > 0.1 or abs(dy) > 0.1:
            ax.arrow(x_coords[0], y_coords[0], dx*0.2, dy*0.2,
                    head_width=8, head_length=6, fc='green', ec='green',
                    linewidth=2, zorder=13, alpha=0.8)
    
    # Punto final real
    ax.scatter(x_coords[-1], y_coords[-1], c='red', s=350, marker='s', 
              edgecolors='darkred', linewidths=3,
              label='Final Real', zorder=12, alpha=0.9)
    
    # Flechas de dirección a lo largo de la trayectoria
    step = max(1, len(data) // 25)
    for i in range(0, len(data)-1, step):
        if i + 1 < len(data):
            dx = x_coords[i+1] - x_coords[i]
            dy = y_coords[i+1] - y_coords[i]
            if abs(dx) > 0.5 or abs(dy) > 0.5:
                ax.arrow(x_coords[i], y_coords[i], dx*0.3, dy*0.3,
                        head_width=5, head_length=3, fc='blue', ec='blue',
                        alpha=0.4, zorder=6, length_includes_head=True)
    
    # Colorbar para velocidad
    cbar = plt.colorbar(scatter, ax=ax, label='Velocidad (cm/s)', pad=0.02)
    cbar.ax.tick_params(labelsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # ANOTACIONES Y ETIQUETAS
    # ═══════════════════════════════════════════════════════════
    
    # Calcular métricas para mostrar
    metrics = calculate_metrics(data)
    initial_dist = data[0]['distance_cm'] if data else 0
    final_dist = data[-1]['distance_cm'] if data else 0
    
    # Texto informativo
    info_text = (
        f"Tipo Potencial: {metadata['potential_type'].upper()}\n"
        f"Tiempo total: {metrics['total_time_s']:.2f} s\n"
        f"Distancia recorrida: {metrics['total_distance_traveled_cm']:.2f} cm\n"
        f"Distancia final: {final_dist:.2f} cm\n"
        f"Eficiencia: {metrics['efficiency']*100:.1f}%\n"
        f"Velocidad promedio: {metrics['avg_velocity_cm_s']:.2f} cm/s"
    )
    
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
           fontsize=10, verticalalignment='top',
           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
           family='monospace')
    
    # Etiquetas y título
    ax.set_xlabel('X (cm)', fontsize=13, fontweight='bold')
    ax.set_ylabel('Y (cm)', fontsize=13, fontweight='bold')
    ax.set_title(
        f'MAPA DE NAVEGACIÓN - {metadata["filename"]}\n'
        f'Comparación: Planificado vs Real',
        fontsize=16, fontweight='bold', pad=20
    )
    
    # Leyenda
    ax.legend(loc='upper right', fontsize=10, framealpha=0.9, 
             fancybox=True, shadow=True)
    
    ax.set_aspect('equal', adjustable='box')
    
    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight', facecolor='white')
        print(f"Mapa guardado: {save_path}")
    else:
        plt.show()


def plot_comparison(all_data: List[List[Dict]], all_metadata: List[Dict], 
                    save_path: Optional[Path] = None):
    """
    Genera gráficos comparativos entre múltiples ejecuciones.
    
    Args:
        all_data: Lista de listas de datos de navegación
        all_metadata: Lista de metadatos
        save_path: Ruta opcional para guardar el gráfico
    """
    if not HAS_MATPLOTLIB:
        print("[WARNING] matplotlib no disponible. No se puede generar gráfico comparativo.")
        return
    
    # Colores para diferentes ejecuciones
    colors = plt.cm.tab10(np.linspace(0, 1, len(all_data)))
    
    fig, axes = plt.subplots(2, 2, figsize=(18, 14))
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 1: Trayectorias superpuestas
    # ═══════════════════════════════════════════════════════════
    ax1 = axes[0, 0]
    for idx, (data, metadata) in enumerate(zip(all_data, all_metadata)):
        x_coords = [row['x_cm'] for row in data]
        y_coords = [row['y_cm'] for row in data]
        label = f"{metadata['potential_type']} ({metadata['filename'][:20]}...)"
        ax1.plot(x_coords, y_coords, color=colors[idx], linewidth=2.5, 
                label=label, alpha=0.7)
        ax1.scatter(x_coords[0], y_coords[0], c=colors[idx], s=100, 
                   marker='o', zorder=10, edgecolors='black', linewidths=2)
        ax1.scatter(x_coords[-1], y_coords[-1], c=colors[idx], s=100, 
                   marker='s', zorder=10, edgecolors='black', linewidths=2)
    
    ax1.set_xlabel('X (cm)', fontsize=12)
    ax1.set_ylabel('Y (cm)', fontsize=12)
    ax1.set_title('Comparación de Trayectorias', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=9, loc='best')
    ax1.set_aspect('equal', adjustable='box')
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 2: Distancia al objetivo normalizada por tiempo
    # ═══════════════════════════════════════════════════════════
    ax2 = axes[0, 1]
    for idx, (data, metadata) in enumerate(zip(all_data, all_metadata)):
        times = [row['elapsed_s'] for row in data]
        distances = [row['distance_cm'] for row in data]
        # Normalizar tiempo a 0-100%
        if times[-1] > 0:
            normalized_times = [t / times[-1] * 100 for t in times]
        else:
            normalized_times = times
        label = f"{metadata['potential_type']}"
        ax2.plot(normalized_times, distances, color=colors[idx], linewidth=2.5, 
                label=label, alpha=0.8)
    
    ax2.set_xlabel('Tiempo Normalizado (%)', fontsize=12)
    ax2.set_ylabel('Distancia al objetivo (cm)', fontsize=12)
    ax2.set_title('Evolución de Distancia (Normalizada)', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 3: Velocidades promedio comparadas
    # ═══════════════════════════════════════════════════════════
    ax3 = axes[1, 0]
    for idx, (data, metadata) in enumerate(zip(all_data, all_metadata)):
        times = [row['elapsed_s'] for row in data]
        velocities = [abs(row['v_linear']) for row in data]
        label = f"{metadata['potential_type']}"
        ax3.plot(times, velocities, color=colors[idx], linewidth=2, 
                label=label, alpha=0.7)
    
    ax3.set_xlabel('Tiempo (s)', fontsize=12)
    ax3.set_ylabel('Velocidad Lineal (cm/s)', fontsize=12)
    ax3.set_title('Comparación de Velocidades', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=9)
    
    # ═══════════════════════════════════════════════════════════
    # SUBPLOT 4: Métricas comparativas (barras)
    # ═══════════════════════════════════════════════════════════
    ax4 = axes[1, 1]
    
    # Calcular métricas para cada ejecución
    all_metrics = [calculate_metrics(data) for data in all_data]
    
    metrics_to_compare = ['total_time_s', 'final_distance_cm', 'efficiency', 'avg_velocity_cm_s']
    metric_labels = ['Tiempo (s)', 'Dist. Final (cm)', 'Eficiencia', 'Vel. Prom (cm/s)']
    
    x_pos = np.arange(len(all_metadata))
    width = 0.2
    
    for i, (metric_key, metric_label) in enumerate(zip(metrics_to_compare, metric_labels)):
        values = [metrics[metric_key] for metrics in all_metrics]
        # Normalizar valores para mejor visualización (excepto eficiencia)
        if metric_key != 'efficiency':
            max_val = max(values) if values else 1
            if max_val > 0:
                normalized_values = [v / max_val for v in values]
            else:
                normalized_values = values
        else:
            normalized_values = values
        
        offset = (i - len(metrics_to_compare)/2) * width + width/2
        ax4.bar(x_pos + offset, normalized_values, width, 
               label=metric_label, alpha=0.8, color=colors[i % len(colors)])
    
    ax4.set_xlabel('Ejecuciones', fontsize=12)
    ax4.set_ylabel('Valores Normalizados', fontsize=12)
    ax4.set_title('Métricas Comparativas Normalizadas', fontsize=14, fontweight='bold')
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels([m['potential_type'] for m in all_metadata], fontsize=9)
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3, axis='y')
    
    plt.suptitle(
        f'COMPARACIÓN DE {len(all_data)} EJECUCIONES',
        fontsize=16, fontweight='bold', y=0.995
    )
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight', facecolor='white')
        print(f"Gráfico comparativo guardado: {save_path}")
    else:
        plt.show()


def compare_logs(filepaths: List[Path], output_dir: Optional[Path] = None, 
                 plot_comparison_flag: bool = False):
    """
    Compara múltiples archivos CSV y genera un resumen comparativo.
    
    Args:
        filepaths: Lista de rutas a archivos CSV
        output_dir: Directorio opcional para guardar resultados
        plot_comparison_flag: Si True, genera gráficos comparativos
    """
    all_metrics = []
    all_metadata = []
    all_data = []
    
    print("\n" + "="*70)
    print("ANÁLISIS COMPARATIVO DE LOGS")
    print("="*70)
    
    for filepath in filepaths:
        data, metadata = load_csv(filepath)
        metrics = calculate_metrics(data)
        all_metrics.append(metrics)
        all_metadata.append(metadata)
        all_data.append(data)
        print(f"Procesado: {metadata['filename']}")
    
    # Tabla comparativa mejorada
    print("\n" + "="*70)
    print("TABLA COMPARATIVA DETALLADA")
    print("="*70)
    
    # Encabezados expandidos
    print(f"{'Archivo':<35} {'Tipo':<10} {'Tiempo':<10} {'Dist.Final':<12} "
          f"{'Eficiencia':<12} {'Vel.Prom':<12} {'Error Final':<12}")
    print("-" * 100)
    
    for metadata, metrics in zip(all_metadata, all_metrics):
        filename_short = metadata['filename'][:32] + "..." if len(metadata['filename']) > 35 else metadata['filename']
        potential_type = metadata['potential_type'][:8]
        print(f"{filename_short:<35} {potential_type:<10} {metrics['total_time_s']:<10.2f} "
              f"{metrics['final_distance_cm']:<12.2f} {metrics['efficiency']:<12.3f} "
              f"{metrics['avg_velocity_cm_s']:<12.2f} {metrics['final_error_cm']:<12.2f}")
    
    # Estadísticas agregadas
    print("\n" + "="*70)
    print("ESTADÍSTICAS AGREGADAS")
    print("="*70)
    
    if all_metrics:
        avg_time = sum(m['total_time_s'] for m in all_metrics) / len(all_metrics)
        avg_dist = sum(m['final_distance_cm'] for m in all_metrics) / len(all_metrics)
        avg_eff = sum(m['efficiency'] for m in all_metrics) / len(all_metrics)
        avg_vel = sum(m['avg_velocity_cm_s'] for m in all_metrics) / len(all_metrics)
        
        best_time_idx = min(range(len(all_metrics)), key=lambda i: all_metrics[i]['total_time_s'])
        best_dist_idx = min(range(len(all_metrics)), key=lambda i: all_metrics[i]['final_distance_cm'])
        best_eff_idx = max(range(len(all_metrics)), key=lambda i: all_metrics[i]['efficiency'])
        
        print(f"Promedio tiempo:        {avg_time:.2f} s")
        print(f"Promedio distancia:     {avg_dist:.2f} cm")
        print(f"Promedio eficiencia:    {avg_eff:.3f}")
        print(f"Promedio velocidad:     {avg_vel:.2f} cm/s")
        print(f"\nMejor tiempo:           {all_metadata[best_time_idx]['filename']}")
        print(f"Mejor distancia final:  {all_metadata[best_dist_idx]['filename']}")
        print(f"Mejor eficiencia:       {all_metadata[best_eff_idx]['filename']}")
    
    # Exportar CSV comparativo si se solicita
    if output_dir:
        output_dir.mkdir(exist_ok=True)
        csv_path = output_dir / "comparison_summary.csv"
        
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['filename', 'potential_type', 'total_time_s', 'total_time_min',
                           'initial_distance_cm', 'final_distance_cm', 'total_distance_traveled_cm',
                           'efficiency', 'avg_velocity_cm_s', 'max_velocity_cm_s',
                           'final_error_cm', 'avg_distance_error_cm', 'avg_angle_error_deg'])
            
            for metadata, metrics in zip(all_metadata, all_metrics):
                writer.writerow([
                    metadata['filename'],
                    metadata['potential_type'],
                    metrics['total_time_s'],
                    metrics['total_time_min'],
                    metrics['initial_distance_cm'],
                    metrics['final_distance_cm'],
                    metrics['total_distance_traveled_cm'],
                    metrics['efficiency'],
                    metrics['avg_velocity_cm_s'],
                    metrics['max_velocity_cm_s'],
                    metrics['final_error_cm'],
                    metrics['avg_distance_error_cm'],
                    metrics['avg_angle_error_deg']
                ])
        
        print(f"\nResumen comparativo guardado: {csv_path}")
    
    # Generar gráficos comparativos si se solicita
    if plot_comparison_flag and HAS_MATPLOTLIB:
        comparison_path = None
        if output_dir:
            comparison_path = output_dir / "comparison_plots.png"
        plot_comparison(all_data, all_metadata, comparison_path)
    
    print("="*70 + "\n")


# ══════════════════════════════════════════════════════════
#  FUNCIONES DE MENÚ INTERACTIVO
# ══════════════════════════════════════════════════════════

def find_csv_files(search_dirs: List[Path] = None) -> List[Path]:
    """
    Busca archivos CSV en directorios comunes.
    
    Args:
        search_dirs: Lista de directorios donde buscar. Si es None, usa ubicaciones comunes.
        
    Returns:
        Lista de rutas a archivos CSV encontrados
    """
    if search_dirs is None:
        search_dirs = [
            Path('logs'),
            Path('PL5/logs'),
            Path('../logs'),
            Path('.'),
        ]
    
    csv_files = []
    for search_dir in search_dirs:
        if search_dir.exists() and search_dir.is_dir():
            found = list(search_dir.glob("velocities_*.csv"))
            csv_files.extend(found)
    
    # Eliminar duplicados y ordenar
    csv_files = sorted(set(csv_files), key=lambda p: p.name)
    return csv_files


def display_file_menu(csv_files: List[Path]) -> List[int]:
    """
    Muestra un menú para seleccionar archivos CSV.
    
    Args:
        csv_files: Lista de archivos CSV disponibles
        
    Returns:
        Lista de índices seleccionados (0-based)
    """
    if not csv_files:
        print("\n[ERROR] No se encontraron archivos CSV (velocities_*.csv)")
        print("        Buscando en: logs/, PL5/logs/, ../logs/")
        return []
    
    print("\n" + "="*80)
    print("ARCHIVOS CSV DISPONIBLES")
    print("="*80)
    
    for idx, csv_file in enumerate(csv_files, 1):
        # Obtener información básica del archivo
        try:
            size_kb = csv_file.stat().st_size / 1024
            print(f"  [{idx:2d}] {csv_file.name:<60} ({size_kb:.1f} KB)")
        except:
            print(f"  [{idx:2d}] {csv_file.name:<60}")
    
    print("="*80)
    print("\nOpciones:")
    print("  - Ingresa números separados por comas para seleccionar múltiples archivos (ej: 1,3,5)")
    print("  - Ingresa 'all' para seleccionar todos")
    print("  - Ingresa 'q' para salir")
    
    while True:
        try:
            selection = input("\nSelecciona archivo(s): ").strip().lower()
            
            if selection == 'q':
                return []
            
            if selection == 'all':
                return list(range(len(csv_files)))
            
            # Parsear selección
            indices = []
            for part in selection.split(','):
                part = part.strip()
                if part:
                    idx = int(part) - 1  # Convertir a 0-based
                    if 0 <= idx < len(csv_files):
                        indices.append(idx)
                    else:
                        print(f"[ERROR] Número {part} fuera de rango (1-{len(csv_files)})")
                        raise ValueError
            
            if indices:
                return sorted(set(indices))  # Eliminar duplicados y ordenar
            else:
                print("[ERROR] No se seleccionó ningún archivo válido")
                
        except ValueError:
            print("[ERROR] Entrada inválida. Intenta de nuevo.")
        except KeyboardInterrupt:
            print("\n\n[INFO] Operación cancelada por el usuario")
            return []


def display_options_menu() -> Dict[str, bool]:
    """
    Muestra un menú de opciones de visualización.
    
    Returns:
        Diccionario con las opciones seleccionadas
    """
    options = {
        'plot': False,
        'map': False,
        'compare': False,
        'compare_plots': False,
        'save_plots': False,
    }
    
    print("\n" + "="*80)
    print("OPCIONES DE VISUALIZACIÓN")
    print("="*80)
    print("  [1] Gráficos de trayectoria y métricas (dashboard completo)")
    print("  [2] Mapa con entorno planificado y trayectoria real")
    print("  [3] Comparación entre archivos (estadísticas)")
    print("  [4] Gráficos comparativos visuales (requiere múltiples archivos)")
    print("  [5] Guardar gráficos en archivos")
    print("  [6] Seleccionar todas las opciones")
    print("  [7] Continuar sin gráficos (solo métricas en texto)")
    print("="*80)
    print("\nIngresa números separados por comas (ej: 1,2,5) o 'q' para salir")
    
    while True:
        try:
            selection = input("\nSelecciona opciones: ").strip().lower()
            
            if selection == 'q':
                return {}
            
            if selection == '7':
                return options  # Sin gráficos
            
            if selection == '6':
                # Seleccionar todas
                options['plot'] = True
                options['map'] = True
                options['compare'] = True
                options['compare_plots'] = True
                options['save_plots'] = True
                return options
            
            # Parsear selección
            selected = set()
            for part in selection.split(','):
                part = part.strip()
                if part:
                    num = int(part)
                    if 1 <= num <= 5:
                        selected.add(num)
                    else:
                        print(f"[ERROR] Número {part} fuera de rango (1-7)")
                        raise ValueError
            
            if selected:
                if 1 in selected:
                    options['plot'] = True
                if 2 in selected:
                    options['map'] = True
                if 3 in selected:
                    options['compare'] = True
                if 4 in selected:
                    options['compare_plots'] = True
                if 5 in selected:
                    options['save_plots'] = True
                
                return options
            else:
                print("[ERROR] No se seleccionó ninguna opción válida")
                
        except ValueError:
            print("[ERROR] Entrada inválida. Intenta de nuevo.")
        except KeyboardInterrupt:
            print("\n\n[INFO] Operación cancelada por el usuario")
            return {}


def get_output_directory() -> Optional[Path]:
    """
    Solicita al usuario el directorio de salida.
    
    Returns:
        Path al directorio o None si no se especifica
    """
    print("\n" + "="*80)
    print("DIRECTORIO DE SALIDA")
    print("="*80)
    print("  Ingresa la ruta del directorio donde guardar los gráficos")
    print("  (presiona Enter para usar 'analysis_output' o 'q' para no guardar)")
    
    while True:
        try:
            response = input("\nDirectorio: ").strip()
            
            if response.lower() == 'q' or not response:
                if not response:
                    return Path('analysis_output')
                return None
            
            output_dir = Path(response)
            return output_dir
                
        except KeyboardInterrupt:
            print("\n\n[INFO] Operación cancelada por el usuario")
            return None


def interactive_menu():
    """
    Menú interactivo principal para seleccionar archivos y opciones.
    """
    print("\n" + "="*80)
    print("ANALIZADOR DE LOGS CSV - ROBOT CREATE 3")
    print("="*80)
    print("Autores: Alan Ariel Salazar, Yago Ramos Sánchez")
    print("Institución: UIE - Universidad Intercontinental de la Empresa")
    print("="*80)
    
    # Buscar archivos CSV
    print("\n[INFO] Buscando archivos CSV...")
    csv_files = find_csv_files()
    
    if not csv_files:
        print("\n[ERROR] No se encontraron archivos CSV.")
        print("        Asegúrate de tener archivos velocities_*.csv en:")
        print("        - logs/")
        print("        - PL5/logs/")
        return
    
    # Seleccionar archivos
    selected_indices = display_file_menu(csv_files)
    
    if not selected_indices:
        print("\n[INFO] No se seleccionaron archivos. Saliendo...")
        return
    
    selected_files = [csv_files[i] for i in selected_indices]
    
    print(f"\n[INFO] Archivos seleccionados: {len(selected_files)}")
    for f in selected_files:
        print(f"       - {f.name}")
    
    # Seleccionar opciones
    options = display_options_menu()
    
    if not options:
        print("\n[INFO] No se seleccionaron opciones. Saliendo...")
        return
    
    # Si se requiere guardar, preguntar por directorio
    output_dir = None
    if options.get('save_plots', False):
        output_dir = get_output_directory()
        if output_dir:
            output_dir.mkdir(exist_ok=True)
            print(f"\n[INFO] Los gráficos se guardarán en: {output_dir}")
    
    # Cargar mapa si se solicita
    map_data = None
    if options.get('map', False):
        print("\n[INFO] Cargando datos del mapa...")
        map_data = load_map_data()
        if map_data:
            print(f"[INFO] Mapa cargado correctamente")
        else:
            print(f"[WARNING] No se encontró el archivo del mapa (data/points.json)")
            print(f"          Se mostrará solo la trayectoria real")
    
    # Procesar archivos
    print("\n" + "="*80)
    print("PROCESANDO ARCHIVOS...")
    print("="*80)
    
    # Modo comparativo si hay múltiples archivos
    if len(selected_files) > 1:
        if options.get('compare', False):
            compare_logs(selected_files, output_dir, 
                        plot_comparison_flag=options.get('compare_plots', False))
        
        # Gráficos individuales
        if options.get('plot', False) and HAS_MATPLOTLIB:
            print("\n[INFO] Generando gráficos individuales...")
            for filepath in selected_files:
                data, metadata = load_csv(filepath)
                if options.get('save_plots', False) and output_dir:
                    traj_path = output_dir / f"{metadata['filename']}_trajectory.png"
                    metrics_path = output_dir / f"{metadata['filename']}_metrics.png"
                    plot_trajectory(data, metadata, traj_path)
                    plot_metrics(data, metadata, metrics_path)
                else:
                    plot_trajectory(data, metadata)
                    plot_metrics(data, metadata)
        
        # Mapas
        if options.get('map', False) and HAS_MATPLOTLIB:
            print("\n[INFO] Generando mapas...")
            for filepath in selected_files:
                data, metadata = load_csv(filepath)
                if options.get('save_plots', False) and output_dir:
                    map_path = output_dir / f"{metadata['filename']}_map.png"
                    plot_map_with_trajectory(data, metadata, map_data, map_path)
                else:
                    plot_map_with_trajectory(data, metadata, map_data)
    
    else:
        # Modo análisis individual
        filepath = selected_files[0]
        data, metadata = load_csv(filepath)
        metrics = calculate_metrics(data)
        
        print_metrics(metadata, metrics)
        
        if options.get('plot', False) and HAS_MATPLOTLIB:
            print("\n[INFO] Generando gráficos...")
            if options.get('save_plots', False) and output_dir:
                traj_path = output_dir / f"{metadata['filename']}_trajectory.png"
                metrics_path = output_dir / f"{metadata['filename']}_metrics.png"
                plot_trajectory(data, metadata, traj_path)
                plot_metrics(data, metadata, metrics_path)
            else:
                plot_trajectory(data, metadata)
                plot_metrics(data, metadata)
        
        if options.get('map', False) and HAS_MATPLOTLIB:
            print("\n[INFO] Generando mapa...")
            if options.get('save_plots', False) and output_dir:
                map_path = output_dir / f"{metadata['filename']}_map.png"
                plot_map_with_trajectory(data, metadata, map_data, map_path)
            else:
                plot_map_with_trajectory(data, metadata, map_data)
    
    print("\n" + "="*80)
    print("ANÁLISIS COMPLETADO")
    print("="*80)
    if output_dir:
        print(f"Gráficos guardados en: {output_dir.absolute()}")
    print("="*80 + "\n")


# ══════════════════════════════════════════════════════════
#  FUNCIÓN PRINCIPAL
# ══════════════════════════════════════════════════════════

def main():
    """Función principal del analizador de logs."""
    parser = argparse.ArgumentParser(
        description="Analiza archivos CSV de navegación del robot Create 3",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos de uso:
  # Analizar un archivo específico
  python utils/analyze_logs.py logs/velocities_conic_combined_20251113_171126.csv
  
  # Analizar con visualización completa (dashboard de 9 gráficos)
  python utils/analyze_logs.py logs/velocities_conic_combined_20251113_171126.csv --plot
  
  # Guardar gráficos en archivos
  python utils/analyze_logs.py logs/velocities_conic_combined_20251113_171126.csv --plot --save-plots --output analysis_output
  
  # Comparar múltiples archivos con estadísticas
  python utils/analyze_logs.py logs/ --compare
  
  # Comparar con gráficos comparativos visuales
  python utils/analyze_logs.py logs/ --compare --compare-plots
  
  # Comparar y guardar todos los gráficos
  python utils/analyze_logs.py logs/ --compare --compare-plots --plot --save-plots --output analysis_output
  
  # Generar mapa con entorno planificado y trayectoria real
  python utils/analyze_logs.py logs/velocities_conic_combined_20251113_171126.csv --map
  
  # Generar mapa y guardarlo
  python utils/analyze_logs.py logs/velocities_conic_combined_20251113_171126.csv --map --save-plots --output analysis_output
  
  # Analizar con mapa y todos los gráficos
  python utils/analyze_logs.py logs/velocities_conic_combined_20251113_171126.csv --plot --map --save-plots
  
  # Analizar todos los archivos en un directorio
  python utils/analyze_logs.py logs/ --all
        """
    )
    
    parser.add_argument(
        'input',
        type=str,
        nargs='?',  # Hacer opcional para modo interactivo
        help='Archivo CSV o directorio con archivos CSV (opcional si se usa --interactive)'
    )
    parser.add_argument(
        '--compare',
        action='store_true',
        help='Comparar múltiples archivos CSV'
    )
    parser.add_argument(
        '--plot',
        action='store_true',
        help='Generar gráficos de trayectoria y métricas'
    )
    parser.add_argument(
        '--all',
        action='store_true',
        help='Analizar todos los archivos CSV en el directorio'
    )
    parser.add_argument(
        '--output',
        type=str,
        default=None,
        help='Directorio para guardar resultados (solo con --compare)'
    )
    parser.add_argument(
        '--save-plots',
        action='store_true',
        help='Guardar gráficos en archivos (requiere --plot)'
    )
    parser.add_argument(
        '--compare-plots',
        action='store_true',
        help='Generar gráficos comparativos entre múltiples archivos (requiere --compare)'
    )
    parser.add_argument(
        '--map',
        action='store_true',
        help='Generar mapa con entorno planificado y trayectoria real (usa data/points.json)'
    )
    parser.add_argument(
        '--map-file',
        type=str,
        default=None,
        help='Ruta al archivo JSON del mapa (por defecto busca data/points.json)'
    )
    parser.add_argument(
        '--interactive',
        action='store_true',
        help='Iniciar menú interactivo (ignora otros argumentos)'
    )
    
    args = parser.parse_args()
    
    # Si se solicita modo interactivo o no hay argumentos, mostrar menú
    if args.interactive or (not args.input and len(sys.argv) == 1):
        interactive_menu()
        return
    
    # Validar que se proporcionó input si no es modo interactivo
    if not args.input:
        print("[ERROR] Se requiere especificar un archivo CSV o directorio, o usar --interactive")
        parser.print_help()
        sys.exit(1)
    
    input_path = Path(args.input)
    
    # Determinar archivos a procesar
    if input_path.is_file():
        filepaths = [input_path]
    elif input_path.is_dir():
        if args.all or args.compare:
            filepaths = list(input_path.glob("velocities_*.csv"))
            if not filepaths:
                print(f"[ERROR] No se encontraron archivos CSV en {input_path}")
                sys.exit(1)
        else:
            print(f"[ERROR] {input_path} es un directorio. Usa --all o --compare")
            sys.exit(1)
    else:
        print(f"[ERROR] {input_path} no existe")
        sys.exit(1)
    
    # Cargar datos del mapa si se solicita
    map_data = None
    if args.map:
        map_json_path = Path(args.map_file) if args.map_file else None
        map_data = load_map_data(map_json_path)
        if map_data:
            print(f"[INFO] Mapa cargado desde {map_json_path or 'ubicación automática'}")
        else:
            print(f"[WARNING] No se encontró el archivo del mapa. Se mostrará solo la trayectoria.")
    
    # Procesar archivos
    if args.compare or len(filepaths) > 1:
        # Modo comparativo
        output_dir = Path(args.output) if args.output else None
        compare_logs(filepaths, output_dir, plot_comparison_flag=args.compare_plots)
        
        # Generar gráficos individuales si se solicita
        if args.plot and HAS_MATPLOTLIB:
            for filepath in filepaths:
                data, metadata = load_csv(filepath)
                metrics = calculate_metrics(data)
                
                if args.save_plots:
                    plot_dir = Path(args.output) if args.output else Path('analysis_output')
                    plot_dir.mkdir(exist_ok=True)
                    traj_path = plot_dir / f"{metadata['filename']}_trajectory.png"
                    metrics_path = plot_dir / f"{metadata['filename']}_metrics.png"
                    plot_trajectory(data, metadata, traj_path)
                    plot_metrics(data, metadata, metrics_path)
                else:
                    plot_trajectory(data, metadata)
                    plot_metrics(data, metadata)
        
        # Generar mapas si se solicita
        if args.map and HAS_MATPLOTLIB:
            for filepath in filepaths:
                data, metadata = load_csv(filepath)
                if args.save_plots:
                    plot_dir = Path(args.output) if args.output else Path('analysis_output')
                    plot_dir.mkdir(exist_ok=True)
                    map_path = plot_dir / f"{metadata['filename']}_map.png"
                    plot_map_with_trajectory(data, metadata, map_data, map_path)
                else:
                    plot_map_with_trajectory(data, metadata, map_data)
    else:
        # Modo análisis individual
        data, metadata = load_csv(filepaths[0])
        metrics = calculate_metrics(data)
        print_metrics(metadata, metrics)
        
        if args.plot and HAS_MATPLOTLIB:
            if args.save_plots:
                plot_dir = Path(args.output) if args.output else Path('analysis_output')
                plot_dir.mkdir(exist_ok=True)
                traj_path = plot_dir / f"{metadata['filename']}_trajectory.png"
                metrics_path = plot_dir / f"{metadata['filename']}_metrics.png"
                plot_trajectory(data, metadata, traj_path)
                plot_metrics(data, metadata, metrics_path)
            else:
                plot_trajectory(data, metadata)
                plot_metrics(data, metadata)
        
        # Generar mapa si se solicita
        if args.map and HAS_MATPLOTLIB:
            if args.save_plots:
                plot_dir = Path(args.output) if args.output else Path('analysis_output')
                plot_dir.mkdir(exist_ok=True)
                map_path = plot_dir / f"{metadata['filename']}_map.png"
                plot_map_with_trajectory(data, metadata, map_data, map_path)
            else:
                plot_map_with_trajectory(data, metadata, map_data)


if __name__ == "__main__":
    main()

