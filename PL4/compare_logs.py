"""
Análisis comparativo de logs: Sistema antiguo vs nuevo

Compara los archivos CSV de telemetría para visualizar
la mejora en el control de velocidad ante obstáculos.

Uso:
    python compare_logs.py
"""

import pandas as pd
import glob
from pathlib import Path


def analyze_log(filepath):
    """Analiza un archivo de log y extrae estadísticas"""
    try:
        df = pd.read_csv(filepath)
        
        stats = {
            'filename': Path(filepath).name,
            'duration_s': float(df['elapsed_s'].max()) if 'elapsed_s' in df.columns else 0,
            'avg_v_linear': df['v_linear'].mean() if 'v_linear' in df.columns else 0,
            'max_v_linear': df['v_linear'].max() if 'v_linear' in df.columns else 0,
            'min_v_linear': df['v_linear'].min() if 'v_linear' in df.columns else 0,
            'total_rows': len(df),
            'obstacles_detected': (df['num_obstacles'] > 0).sum() if 'num_obstacles' in df.columns else 0,
        }
        
        # Analizar niveles de seguridad si existen
        if 'max_ir_front' in df.columns:
            stats['emergency_count'] = (df['max_ir_front'] >= 800).sum()
            stats['critical_count'] = ((df['max_ir_front'] >= 400) & (df['max_ir_front'] < 800)).sum()
            stats['warning_count'] = ((df['max_ir_front'] >= 200) & (df['max_ir_front'] < 400)).sum()
            stats['caution_count'] = ((df['max_ir_front'] >= 100) & (df['max_ir_front'] < 200)).sum()
            stats['clear_count'] = (df['max_ir_front'] < 100).sum()
        
        return stats
    except Exception as e:
        print(f"❌ Error al leer {filepath}: {e}")
        return None


def print_comparison(old_stats, new_stats):
    """Imprime comparación lado a lado"""
    print("\n" + "="*90)
    print("📊 COMPARACIÓN DE RENDIMIENTO")
    print("="*90)
    
    print(f"\n{'Métrica':<30} {'Sistema ANTIGUO':<30} {'Sistema NUEVO':<30}")
    print("-"*90)
    
    # Duración
    print(f"{'Duración total':<30} {old_stats['duration_s']:>8.2f} s {' '*20} "
          f"{new_stats['duration_s']:>8.2f} s")
    
    # Velocidades
    print(f"{'Velocidad promedio':<30} {old_stats['avg_v_linear']:>8.2f} cm/s {' '*17} "
          f"{new_stats['avg_v_linear']:>8.2f} cm/s")
    print(f"{'Velocidad máxima':<30} {old_stats['max_v_linear']:>8.2f} cm/s {' '*17} "
          f"{new_stats['max_v_linear']:>8.2f} cm/s")
    
    # Detecciones
    print(f"{'Obstáculos detectados':<30} {old_stats['obstacles_detected']:>8d} veces {' '*16} "
          f"{new_stats['obstacles_detected']:>8d} veces")
    
    # Niveles de seguridad (solo nuevo)
    if 'emergency_count' in new_stats:
        print("\n" + "-"*90)
        print("Distribución de Niveles de Seguridad (NUEVO sistema):")
        print("-"*90)
        total = new_stats['total_rows']
        print(f"  🚨 EMERGENCIA:   {new_stats['emergency_count']:4d} ({new_stats['emergency_count']/total*100:5.1f}%)")
        print(f"  🔴 CRÍTICO:      {new_stats['critical_count']:4d} ({new_stats['critical_count']/total*100:5.1f}%)")
        print(f"  ⚠️  ADVERTENCIA:  {new_stats['warning_count']:4d} ({new_stats['warning_count']/total*100:5.1f}%)")
        print(f"  ⚡ PRECAUCIÓN:   {new_stats['caution_count']:4d} ({new_stats['caution_count']/total*100:5.1f}%)")
        print(f"  ✅ LIBRE:        {new_stats['clear_count']:4d} ({new_stats['clear_count']/total*100:5.1f}%)")
    
    print("="*90 + "\n")


def main():
    """Función principal"""
    print("\n" + "="*90)
    print("📈 ANÁLISIS COMPARATIVO DE LOGS - Sistema de Seguridad")
    print("="*90)
    
    # Buscar archivos de log
    log_dir = Path("logs")
    if not log_dir.exists():
        print("❌ No se encontró el directorio 'logs'")
        return
    
    csv_files = sorted(log_dir.glob("velocities_*.csv"))
    
    if not csv_files:
        print("❌ No se encontraron archivos CSV en logs/")
        return
    
    print(f"\n📁 Encontrados {len(csv_files)} archivos de log:")
    for i, f in enumerate(csv_files, 1):
        print(f"   {i}. {f.name}")
    
    # Analizar todos los logs
    print("\n⚙️  Analizando logs...\n")
    
    all_stats = []
    for csv_file in csv_files:
        stats = analyze_log(csv_file)
        if stats:
            all_stats.append(stats)
    
    if not all_stats:
        print("❌ No se pudieron analizar logs")
        return
    
    # Mostrar resumen de cada log
    print("\n" + "="*90)
    print("📋 RESUMEN INDIVIDUAL DE LOGS")
    print("="*90)
    
    for stats in all_stats:
        print(f"\n📄 {stats['filename']}")
        print(f"   Duración: {stats['duration_s']:.2f} s")
        print(f"   V_media: {stats['avg_v_linear']:.2f} cm/s  |  V_max: {stats['max_v_linear']:.2f} cm/s")
        print(f"   Obstáculos detectados: {stats['obstacles_detected']} veces")
        
        if 'emergency_count' in stats:
            print(f"   Niveles: 🚨{stats['emergency_count']} 🔴{stats['critical_count']} "
                  f"⚠️{stats['warning_count']} ⚡{stats['caution_count']} ✅{stats['clear_count']}")
    
    print("\n" + "="*90)
    print("💡 RECOMENDACIONES:")
    print("="*90)
    print("1. Los logs con sistema NUEVO deben mostrar:")
    print("   ✓ Menor velocidad promedio ante obstáculos")
    print("   ✓ Distribución clara de niveles de seguridad")
    print("   ✓ Transiciones suaves entre niveles")
    print("\n2. Para análisis gráfico detallado:")
    print("   → Usar analyze_results.py con los CSVs individuales")
    print("="*90 + "\n")


if __name__ == "__main__":
    main()
