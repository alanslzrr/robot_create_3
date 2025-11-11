#!/usr/bin/env python3
"""
Visualización gráfica del sistema de umbrales escalonados

Autores: Alan Salazar, Yago Ramos
Fecha: 4 de noviembre de 2025
Institución: UIE Universidad Intercontinental de la Empresa
Asignatura: Robots Autónomos - Profesor Eladio Dapena
Robot SDK: irobot-edu-sdk (visualización offline)

OBJETIVOS PRINCIPALES:

En este módulo implementamos herramientas de visualización que generan gráficos
y tablas que explican el funcionamiento del sistema de umbrales escalonados de
seguridad. Nuestro objetivo principal era crear visualizaciones claras que
permitieran entender cómo el sistema ajusta la velocidad según la proximidad de
obstáculos detectados mediante sensores IR.

Los objetivos específicos que buscamos alcanzar incluyen:

1. Generar gráficos que muestren la relación entre valores de sensores IR y
   velocidad máxima permitida, facilitando la comprensión del sistema escalonado
2. Visualizar el modelo físico de estimación de distancia basado en la relación
   inversa al cuadrado entre intensidad IR y distancia
3. Crear una tabla de referencia rápida que muestre todos los umbrales, velocidades
   límite y distancias estimadas en un formato visualmente atractivo
4. Guardar todas las visualizaciones como archivos PNG de alta resolución para
   uso en documentación y presentaciones
5. Proporcionar opción de visualización interactiva durante el desarrollo para
   ajustar parámetros si es necesario
"""

import matplotlib.pyplot as plt
import numpy as np
import sys
from pathlib import Path

# Añadir el directorio padre al path para importar módulos
sys.path.insert(0, str(Path(__file__).parent.parent))
from src import config


def ir_to_distance(ir_value):
    """Estima distancia basada en valor IR (modelo 1/d²)"""
    if ir_value < 50:
        return 100.0  # Sin obstáculo
    
    I_ref = 1000.0
    d_ref = 5.0
    
    if ir_value >= I_ref:
        return d_ref
    else:
        d = d_ref * np.sqrt(I_ref / ir_value)
        return np.clip(d, 5.0, 100.0)


def get_vmax_from_ir(ir_value):
    """Retorna velocidad máxima permitida según IR"""
    if ir_value >= config.IR_THRESHOLD_EMERGENCY:
        return config.V_MAX_EMERGENCY
    elif ir_value >= config.IR_THRESHOLD_CRITICAL:
        return config.V_MAX_CRITICAL
    elif ir_value >= config.IR_THRESHOLD_WARNING:
        return config.V_MAX_WARNING
    elif ir_value >= config.IR_THRESHOLD_CAUTION:
        return config.V_MAX_CAUTION
    else:
        return config.V_MAX_CM_S


def get_safety_level(ir_value):
    """Retorna nivel de seguridad según IR"""
    if ir_value >= config.IR_THRESHOLD_EMERGENCY:
        return "EMERGENCIA"
    elif ir_value >= config.IR_THRESHOLD_CRITICAL:
        return "CRÍTICO"
    elif ir_value >= config.IR_THRESHOLD_WARNING:
        return "ADVERTENCIA"
    elif ir_value >= config.IR_THRESHOLD_CAUTION:
        return "PRECAUCIÓN"
    else:
        return "LIBRE"


def plot_ir_vs_vmax():
    """Gráfico IR vs Velocidad Máxima"""
    ir_values = np.linspace(0, 1000, 1000)
    vmax_values = [get_vmax_from_ir(ir) for ir in ir_values]
    
    plt.figure(figsize=(12, 6))
    
    # Pintar zonas de fondo
    plt.axhspan(0, config.V_MAX_EMERGENCY, color='red', alpha=0.2, label='EMERGENCIA')
    plt.axhspan(config.V_MAX_EMERGENCY, config.V_MAX_CRITICAL, color='orange', alpha=0.2, label='CRÍTICO')
    plt.axhspan(config.V_MAX_CRITICAL, config.V_MAX_WARNING, color='yellow', alpha=0.2, label='ADVERTENCIA')
    plt.axhspan(config.V_MAX_WARNING, config.V_MAX_CAUTION, color='lightblue', alpha=0.2, label='PRECAUCIÓN')
    plt.axhspan(config.V_MAX_CAUTION, config.V_MAX_CM_S, color='lightgreen', alpha=0.2, label='LIBRE')
    
    # Línea principal
    plt.plot(ir_values, vmax_values, 'b-', linewidth=2.5, label='V_max permitida')
    
    # Líneas verticales de umbrales
    plt.axvline(config.IR_THRESHOLD_EMERGENCY, color='red', linestyle='--', linewidth=1.5, alpha=0.7, label=f'Umbral EMERG ({config.IR_THRESHOLD_EMERGENCY})')
    plt.axvline(config.IR_THRESHOLD_CRITICAL, color='orange', linestyle='--', linewidth=1.5, alpha=0.7, label=f'Umbral CRÍT ({config.IR_THRESHOLD_CRITICAL})')
    plt.axvline(config.IR_THRESHOLD_WARNING, color='gold', linestyle='--', linewidth=1.5, alpha=0.7, label=f'Umbral ADVER ({config.IR_THRESHOLD_WARNING})')
    plt.axvline(config.IR_THRESHOLD_CAUTION, color='blue', linestyle='--', linewidth=1.5, alpha=0.7, label=f'Umbral PREC ({config.IR_THRESHOLD_CAUTION})')
    
    plt.xlabel('Valor Sensor IR', fontsize=12, fontweight='bold')
    plt.ylabel('Velocidad Máxima Permitida (cm/s)', fontsize=12, fontweight='bold')
    plt.title('Sistema de Umbrales Escalonados: IR → V_max', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.xlim(0, 1000)
    plt.ylim(-5, 55)
    
    # Leyenda fuera del gráfico
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=9)
    plt.tight_layout()
    
    return plt.gcf()


def plot_ir_vs_distance():
    """Gráfico IR vs Distancia Estimada"""
    ir_values = np.linspace(50, 1500, 1000)
    distances = [ir_to_distance(ir) for ir in ir_values]
    
    plt.figure(figsize=(12, 6))
    
    plt.plot(ir_values, distances, 'g-', linewidth=2.5, label='Distancia estimada')
    
    # Zonas de color según umbrales
    plt.axvspan(config.IR_THRESHOLD_EMERGENCY, 1500, color='red', alpha=0.2, label='EMERGENCIA (<5cm)')
    plt.axvspan(config.IR_THRESHOLD_CRITICAL, config.IR_THRESHOLD_EMERGENCY, color='orange', alpha=0.2, label='CRÍTICO (5-10cm)')
    plt.axvspan(config.IR_THRESHOLD_WARNING, config.IR_THRESHOLD_CRITICAL, color='yellow', alpha=0.2, label='ADVERTENCIA (10-20cm)')
    plt.axvspan(config.IR_THRESHOLD_CAUTION, config.IR_THRESHOLD_WARNING, color='lightblue', alpha=0.2, label='PRECAUCIÓN (20-40cm)')
    plt.axvspan(0, config.IR_THRESHOLD_CAUTION, color='lightgreen', alpha=0.2, label='LIBRE (>40cm)')
    
    # Líneas de referencia
    plt.axhline(5, color='red', linestyle=':', alpha=0.7)
    plt.axhline(10, color='orange', linestyle=':', alpha=0.7)
    plt.axhline(20, color='gold', linestyle=':', alpha=0.7)
    plt.axhline(40, color='blue', linestyle=':', alpha=0.7)
    
    plt.xlabel('Valor Sensor IR', fontsize=12, fontweight='bold')
    plt.ylabel('Distancia Estimada (cm)', fontsize=12, fontweight='bold')
    plt.title('Modelo de Sensor IR: Intensidad → Distancia (I ∝ 1/d²)', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.xlim(0, 1500)
    plt.ylim(0, 50)
    
    plt.legend(loc='upper right', fontsize=9)
    plt.tight_layout()
    
    return plt.gcf()


def plot_comparison_table():
    """Tabla comparativa visual"""
    fig, ax = plt.subplots(figsize=(14, 8))
    ax.axis('tight')
    ax.axis('off')
    
    # Datos de la tabla
    data = [
        ['Nivel', 'Umbral IR', 'V_max\n(cm/s)', 'Distancia\nEstimada', 'Tiempo\nReacción*', 'Color'],
        ['🚨 EMERGENCIA', f'≥ {config.IR_THRESHOLD_EMERGENCY}', f'{config.V_MAX_EMERGENCY}', '<5 cm', 'Inmediato', '#ff4444'],
        ['🔴 CRÍTICO', f'≥ {config.IR_THRESHOLD_CRITICAL}', f'{config.V_MAX_CRITICAL}', '5-10 cm', '~2.0 s', '#ff8800'],
        ['⚠️ ADVERTENCIA', f'≥ {config.IR_THRESHOLD_WARNING}', f'{config.V_MAX_WARNING}', '10-20 cm', '~1.0 s', '#ffdd00'],
        ['⚡ PRECAUCIÓN', f'≥ {config.IR_THRESHOLD_CAUTION}', f'{config.V_MAX_CAUTION}', '20-40 cm', '~0.7 s', '#4488ff'],
        ['✅ LIBRE', f'< {config.IR_THRESHOLD_CAUTION}', f'{config.V_MAX_CM_S}', '>40 cm', 'N/A', '#44ff44'],
    ]
    
    # Crear tabla
    table = ax.table(cellText=data, cellLoc='center', loc='center',
                     bbox=[0, 0, 1, 1])
    
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    table.scale(1, 2.5)
    
    # Estilo del header
    for i in range(6):
        cell = table[(0, i)]
        cell.set_facecolor('#333333')
        cell.set_text_props(weight='bold', color='white')
    
    # Colorear filas según nivel
    for i in range(1, 6):
        color = data[i][5]
        for j in range(5):
            cell = table[(i, j)]
            cell.set_facecolor(color)
            cell.set_alpha(0.3)
            if j == 0:  # Primera columna en negrita
                cell.set_text_props(weight='bold')
    
    plt.title('Sistema de Umbrales Escalonados - Referencia Rápida\n(*Tiempo @V_max con 20cm de margen)', 
              fontsize=14, fontweight='bold', pad=20)
    
    # Nota al pie
    plt.figtext(0.5, 0.02, 
                'Basado en calibración real con obstáculos a 5cm del robot\n'
                'Modelo físico: I ∝ 1/d² donde I₀=1000 @ d₀=5cm',
                ha='center', fontsize=9, style='italic', color='gray')
    
    return fig


def main():
    """Generar todos los gráficos"""
    print("\n" + "="*70)
    print("📊 GENERANDO VISUALIZACIONES DEL SISTEMA DE SEGURIDAD")
    print("="*70 + "\n")
    
    # Crear carpeta de imágenes si no existe
    images_dir = Path(__file__).parent.parent / "images"
    images_dir.mkdir(exist_ok=True)
    
    print("1️⃣  Generando gráfico IR vs V_max...")
    fig1 = plot_ir_vs_vmax()
    fig1.savefig(images_dir / 'safety_ir_vs_vmax.png', dpi=300, bbox_inches='tight')
    print(f"   ✅ Guardado: {images_dir / 'safety_ir_vs_vmax.png'}")
    
    print("\n2️⃣  Generando gráfico IR vs Distancia...")
    fig2 = plot_ir_vs_distance()
    fig2.savefig(images_dir / 'safety_ir_vs_distance.png', dpi=300, bbox_inches='tight')
    print(f"   ✅ Guardado: {images_dir / 'safety_ir_vs_distance.png'}")
    
    print("\n3️⃣  Generando tabla comparativa...")
    fig3 = plot_comparison_table()
    fig3.savefig(images_dir / 'safety_table.png', dpi=300, bbox_inches='tight')
    print(f"   ✅ Guardado: {images_dir / 'safety_table.png'}")
    
    print("\n" + "="*70)
    print("✅ Visualizaciones generadas correctamente")
    print("="*70)
    print("\n💡 Abre los archivos PNG para ver los gráficos")
    print("   O ejecuta plt.show() para visualización interactiva\n")
    
    # Mostrar interactivamente (opcional)
    respuesta = input("¿Mostrar gráficos interactivos? (s/n): ")
    if respuesta.lower() == 's':
        plt.show()


if __name__ == "__main__":
    main()
