#!/usr/bin/env python3
"""
Test del Sistema de Umbrales Escalonados de Seguridad

Este script demuestra cómo el nuevo sistema limita la velocidad
dinámicamente según las lecturas de los sensores IR.

Autores: Yago Ramos - Salazar Alan
Fecha: 28 de octubre de 2025
"""

import config

def analyze_ir_reading(max_ir_front):
    """
    Analiza una lectura IR y determina el nivel de seguridad
    
    Args:
        max_ir_front: Valor máximo de los sensores frontales
    
    Returns:
        (safety_level, v_max, description)
    """
    if max_ir_front >= config.IR_THRESHOLD_EMERGENCY:
        return "🚨 EMERGENCIA", config.V_MAX_EMERGENCY, "Obstáculo MUY CERCA (<5cm) - PARAR"
    elif max_ir_front >= config.IR_THRESHOLD_CRITICAL:
        return "🔴 CRÍTICO", config.V_MAX_CRITICAL, "Obstáculo cerca (5-10cm) - Velocidad MÍNIMA"
    elif max_ir_front >= config.IR_THRESHOLD_WARNING:
        return "⚠️  ADVERTENCIA", config.V_MAX_WARNING, "Obstáculo medio (10-20cm) - Velocidad REDUCIDA"
    elif max_ir_front >= config.IR_THRESHOLD_CAUTION:
        return "⚡ PRECAUCIÓN", config.V_MAX_CAUTION, "Obstáculo lejano (20-40cm) - Velocidad LIMITADA"
    else:
        return "✅ LIBRE", config.V_MAX_CM_S, "Sin obstáculos (>40cm) - Velocidad NORMAL"


def print_threshold_table():
    """Imprime tabla de referencia de umbrales"""
    print("\n" + "="*80)
    print("📋 TABLA DE UMBRALES DE SEGURIDAD")
    print("="*80)
    print(f"{'Nivel':<20} {'Umbral IR':<15} {'V_max':<15} {'Distancia Est.':<20}")
    print("-"*80)
    print(f"{'🚨 EMERGENCIA':<20} {'≥ ' + str(config.IR_THRESHOLD_EMERGENCY):<15} "
          f"{str(config.V_MAX_EMERGENCY) + ' cm/s':<15} {'< 5 cm':<20}")
    print(f"{'🔴 CRÍTICO':<20} {'≥ ' + str(config.IR_THRESHOLD_CRITICAL):<15} "
          f"{str(config.V_MAX_CRITICAL) + ' cm/s':<15} {'~5-10 cm':<20}")
    print(f"{'⚠️  ADVERTENCIA':<20} {'≥ ' + str(config.IR_THRESHOLD_WARNING):<15} "
          f"{str(config.V_MAX_WARNING) + ' cm/s':<15} {'~10-20 cm':<20}")
    print(f"{'⚡ PRECAUCIÓN':<20} {'≥ ' + str(config.IR_THRESHOLD_CAUTION):<15} "
          f"{str(config.V_MAX_CAUTION) + ' cm/s':<15} {'~20-40 cm':<20}")
    print(f"{'✅ LIBRE':<20} {'< ' + str(config.IR_THRESHOLD_CAUTION):<15} "
          f"{str(config.V_MAX_CM_S) + ' cm/s':<15} {'> 40 cm':<20}")
    print("="*80 + "\n")


def test_scenarios():
    """Prueba diferentes escenarios de detección"""
    print("\n" + "="*80)
    print("🧪 ESCENARIOS DE PRUEBA")
    print("="*80 + "\n")
    
    test_cases = [
        (0, "Sin obstáculo detectado"),
        (50, "Ruido de sensor (ignorado)"),
        (120, "Obstáculo lejano detectado"),
        (250, "Obstáculo a distancia media"),
        (450, "Obstáculo muy cerca (según calibración a 45°)"),
        (900, "Obstáculo frontal perpendicular (según calibración)"),
        (1200, "Obstáculo extremadamente cerca"),
    ]
    
    for ir_value, description in test_cases:
        level, v_max, action = analyze_ir_reading(ir_value)
        print(f"IR Frontal = {ir_value:4d}  →  {level}")
        print(f"   Escenario: {description}")
        print(f"   Acción: {action}")
        print(f"   V_max permitida: {v_max} cm/s")
        print()


def compare_old_vs_new():
    """Compara comportamiento antiguo vs nuevo"""
    print("\n" + "="*80)
    print("📊 COMPARACIÓN: SISTEMA ANTIGUO vs NUEVO")
    print("="*80 + "\n")
    
    OLD_THRESHOLD_SLOW = 150
    OLD_THRESHOLD_STOP = 300
    
    test_values = [80, 160, 250, 350, 500, 850]
    
    print(f"{'IR Value':<12} {'Sistema ANTIGUO':<30} {'Sistema NUEVO':<40}")
    print("-"*82)
    
    for ir_val in test_values:
        # Antiguo
        if ir_val > OLD_THRESHOLD_STOP:
            old_action = "PARAR (sin control gradual)"
        elif ir_val > OLD_THRESHOLD_SLOW:
            old_action = "REDUCIR (sin límite claro)"
        else:
            old_action = "Libre (48 cm/s)"
        
        # Nuevo
        level, v_max, _ = analyze_ir_reading(ir_val)
        new_action = f"{level}  (v≤{v_max} cm/s)"
        
        print(f"{ir_val:<12} {old_action:<30} {new_action:<40}")
    
    print()


def main():
    """Función principal"""
    print("\n" + "="*80)
    print("🛡️  SISTEMA DE UMBRALES ESCALONADOS DE SEGURIDAD")
    print("="*80)
    print("Versión: 2.0")
    print("Autores: Yago Ramos - Salazar Alan")
    print("Fecha: 28 de octubre de 2025")
    print("="*80)
    
    print_threshold_table()
    test_scenarios()
    compare_old_vs_new()
    
    print("\n" + "="*80)
    print("✅ VENTAJAS DEL NUEVO SISTEMA:")
    print("="*80)
    print("1. ✓ Reacción gradual (4 niveles en lugar de 2)")
    print("2. ✓ Límites de velocidad claros y predecibles")
    print("3. ✓ Tiempo suficiente de frenado ante obstáculos")
    print("4. ✓ Basado en calibración real del robot")
    print("5. ✓ Evita colisiones manteniendo movilidad")
    print("="*80 + "\n")
    
    print("💡 RECOMENDACIÓN:")
    print("   Ejecutar pruebas reales con PRM01_P02.py --potential conic")
    print("   Observar las transiciones de nivel en el logger de sensores")
    print("   Revisar los logs CSV para análisis detallado\n")


if __name__ == "__main__":
    main()
