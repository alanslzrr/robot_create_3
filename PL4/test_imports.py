#!/usr/bin/env python3
"""
Script de pruebas completo para verificar todos los módulos e imports del proyecto

Este script verifica:
1. Imports de todos los módulos principales
2. Imports entre módulos (dependencias)
3. Funciones principales de cada módulo
4. Estructura de datos y configuraciones
5. Archivos necesarios
6. Sintaxis de todos los archivos Python

Autores: Alan Salazar, Yago Ramos
Fecha: 4 de noviembre de 2025
"""

import sys
import importlib
import traceback
from pathlib import Path
import json

# Colores para output
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

def print_success(msg):
    print(f"{Colors.GREEN}[OK]{Colors.RESET} {msg}")

def print_error(msg):
    print(f"{Colors.RED}[ERROR]{Colors.RESET} {msg}")

def print_warning(msg):
    print(f"{Colors.YELLOW}[WARN]{Colors.RESET} {msg}")

def print_info(msg):
    print(f"{Colors.BLUE}[INFO]{Colors.RESET} {msg}")

def print_header(msg):
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*70}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{msg}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*70}{Colors.RESET}\n")

# Contador de errores
errors = []
warnings = []

def test_import(module_name, description=""):
    """Prueba importar un módulo"""
    try:
        module = importlib.import_module(module_name)
        print_success(f"Import de '{module_name}' {description}")
        return module
    except Exception as e:
        error_msg = f"Error importando '{module_name}': {str(e)}"
        print_error(error_msg)
        errors.append(error_msg)
        traceback.print_exc()
        return None

def test_module_attributes(module, expected_attrs, module_name):
    """Verifica que un módulo tenga los atributos esperados"""
    missing = []
    for attr in expected_attrs:
        if not hasattr(module, attr):
            missing.append(attr)
    
    if missing:
        error_msg = f"Módulo '{module_name}' falta atributos: {', '.join(missing)}"
        print_error(error_msg)
        errors.append(error_msg)
    else:
        print_success(f"Módulo '{module_name}' tiene todos los atributos necesarios")

def test_function_call(module, func_name, *args, **kwargs):
    """Prueba llamar una función con argumentos dados"""
    try:
        if not hasattr(module, func_name):
            error_msg = f"Función '{func_name}' no existe en '{module.__name__}'"
            print_error(error_msg)
            errors.append(error_msg)
            return None
        
        func = getattr(module, func_name)
        result = func(*args, **kwargs)
        print_success(f"Función '{func_name}' ejecutada correctamente")
        return result
    except Exception as e:
        error_msg = f"Error ejecutando '{func_name}': {str(e)}"
        print_error(error_msg)
        errors.append(error_msg)
        traceback.print_exc()
        return None

def test_file_exists(filepath, description=""):
    """Verifica que un archivo exista"""
    path = Path(filepath)
    if path.exists():
        print_success(f"Archivo existe: {filepath} {description}")
        return True
    else:
        error_msg = f"Archivo no encontrado: {filepath}"
        print_error(error_msg)
        errors.append(error_msg)
        return False

def test_json_file(filepath):
    """Verifica que un archivo JSON sea válido"""
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        print_success(f"JSON válido: {filepath}")
        return data
    except Exception as e:
        error_msg = f"Error leyendo JSON '{filepath}': {str(e)}"
        print_error(error_msg)
        errors.append(error_msg)
        return None

# ============ PRUEBAS PRINCIPALES ============

def test_src_config():
    """Prueba el módulo src.config"""
    print_header("TEST 1: Módulo src.config")
    
    config = test_import("src.config", "(configuración)")
    if not config:
        return False
    
    # Verificar atributos importantes
    expected_attrs = [
        'BLUETOOTH_NAME', 'V_MAX_CM_S', 'V_MIN_CM_S', 'W_MAX_CM_S',
        'CONTROL_DT', 'TOL_DIST_CM', 'TOL_ANGLE_DEG',
        'K_LINEAR', 'K_QUADRATIC', 'K_CONIC', 'K_EXPONENTIAL', 'K_ANGULAR',
        'K_REPULSIVE', 'D_INFLUENCE', 'D_SAFE',
        'IR_THRESHOLD_EMERGENCY', 'IR_THRESHOLD_CRITICAL', 'IR_THRESHOLD_WARNING',
        'IR_THRESHOLD_CAUTION', 'IR_THRESHOLD_DETECT',
        'V_MAX_EMERGENCY', 'V_MAX_CRITICAL', 'V_MAX_WARNING', 'V_MAX_CAUTION',
        'IR_SENSOR_ANGLES', 'POINTS_FILE'
    ]
    test_module_attributes(config, expected_attrs, "src.config")
    
    # Verificar tipos
    assert isinstance(config.BLUETOOTH_NAME, str), "BLUETOOTH_NAME debe ser string"
    assert isinstance(config.V_MAX_CM_S, (int, float)), "V_MAX_CM_S debe ser numérico"
    assert isinstance(config.IR_SENSOR_ANGLES, dict), "IR_SENSOR_ANGLES debe ser dict"
    
    print_success("Configuración básica correcta")
    return True

def test_src_potential_fields():
    """Prueba el módulo src.potential_fields"""
    print_header("TEST 2: Módulo src.potential_fields")
    
    pf = test_import("src.potential_fields", "(potential fields)")
    if not pf:
        return False
    
    # Verificar funciones principales
    expected_funcs = [
        'reset_velocity_ramp', '_wrap_pi', 'attractive_wheel_speeds',
        'ir_sensors_to_obstacles', 'repulsive_force', 'combined_potential_speeds'
    ]
    for func_name in expected_funcs:
        if not hasattr(pf, func_name):
            error_msg = f"Función '{func_name}' no existe en potential_fields"
            print_error(error_msg)
            errors.append(error_msg)
        else:
            print_success(f"Función '{func_name}' existe")
    
    # Probar funciones con datos de prueba
    print_info("Probando funciones con datos de prueba...")
    
    # Test _wrap_pi
    test_function_call(pf, '_wrap_pi', 3.14159)
    test_function_call(pf, '_wrap_pi', -3.14159)
    
    # Test reset_velocity_ramp
    test_function_call(pf, 'reset_velocity_ramp')
    
    # Test attractive_wheel_speeds
    q = (0.0, 0.0, 0.0)
    q_goal = (100.0, 100.0)
    result = test_function_call(pf, 'attractive_wheel_speeds', q, q_goal, potential_type='linear')
    if result:
        v_left, v_right, distance, info = result
        assert isinstance(v_left, (int, float)), "v_left debe ser numérico"
        assert isinstance(v_right, (int, float)), "v_right debe ser numérico"
        assert isinstance(distance, (int, float)), "distance debe ser numérico"
        assert isinstance(info, dict), "info debe ser dict"
        print_success("attractive_wheel_speeds retorna estructura correcta")
    
    # Test ir_sensors_to_obstacles
    ir_sensors = [0, 0, 0, 0, 0, 0, 0]
    result = test_function_call(pf, 'ir_sensors_to_obstacles', q, ir_sensors)
    assert isinstance(result, list), "ir_sensors_to_obstacles debe retornar lista"
    
    # Test repulsive_force
    result = test_function_call(pf, 'repulsive_force', q, ir_sensors)
    if result:
        fx, fy = result
        assert isinstance(fx, (int, float)), "fx debe ser numérico"
        assert isinstance(fy, (int, float)), "fy debe ser numérico"
    
    # Test combined_potential_speeds
    result = test_function_call(pf, 'combined_potential_speeds', q, q_goal, ir_sensors, potential_type='linear')
    if result:
        v_left, v_right, distance, info = result
        assert isinstance(v_left, (int, float)), "v_left debe ser numérico"
        assert isinstance(v_right, (int, float)), "v_right debe ser numérico"
    
    return True

def test_src_safety():
    """Prueba el módulo src.safety"""
    print_header("TEST 3: Módulo src.safety")
    
    safety = test_import("src.safety", "(safety)")
    if not safety:
        return False
    
    # Verificar funciones
    expected_funcs = [
        'saturate_wheel_speeds', 'detect_obstacle', 'check_bumpers',
        'emergency_stop_needed', 'apply_obstacle_slowdown'
    ]
    for func_name in expected_funcs:
        if not hasattr(safety, func_name):
            error_msg = f"Función '{func_name}' no existe en safety"
            print_error(error_msg)
            errors.append(error_msg)
        else:
            print_success(f"Función '{func_name}' existe")
    
    # Probar funciones
    print_info("Probando funciones con datos de prueba...")
    
    # Test saturate_wheel_speeds
    result = test_function_call(safety, 'saturate_wheel_speeds', 50.0, 50.0)
    if result:
        v_left, v_right = result
        assert isinstance(v_left, (int, float)), "v_left debe ser numérico"
        assert isinstance(v_right, (int, float)), "v_right debe ser numérico"
    
    # Test detect_obstacle
    ir_sensors = [0, 0, 0, 0, 0, 0, 0]
    result = test_function_call(safety, 'detect_obstacle', ir_sensors)
    if result:
        assert isinstance(result, dict), "detect_obstacle debe retornar dict"
        assert 'stop' in result, "resultado debe tener 'stop'"
        assert 'slow' in result, "resultado debe tener 'slow'"
    
    # Test check_bumpers
    result = test_function_call(safety, 'check_bumpers', (False, False))
    if result:
        assert isinstance(result, dict), "check_bumpers debe retornar dict"
    
    # Test emergency_stop_needed
    result = test_function_call(safety, 'emergency_stop_needed', (False, False))
    assert isinstance(result, bool), "emergency_stop_needed debe retornar bool"
    
    return True

def test_src_sensor_logger():
    """Prueba el módulo src.sensor_logger"""
    print_header("TEST 4: Módulo src.sensor_logger")
    
    sensor_logger = test_import("src.sensor_logger", "(sensor logger)")
    if not sensor_logger:
        return False
    
    # Verificar clases y funciones
    assert hasattr(sensor_logger, 'SensorLogger'), "Debe tener clase SensorLogger"
    print_success("Clase SensorLogger existe")
    
    # Verificar que tiene get_sensor_snapshot
    assert hasattr(sensor_logger, 'get_sensor_snapshot'), "Debe tener función get_sensor_snapshot"
    print_success("Función get_sensor_snapshot existe")
    
    # Verificar métodos de SensorLogger
    assert hasattr(sensor_logger.SensorLogger, '__init__'), "SensorLogger debe tener __init__"
    assert hasattr(sensor_logger.SensorLogger, 'start'), "SensorLogger debe tener start"
    assert hasattr(sensor_logger.SensorLogger, 'stop'), "SensorLogger debe tener stop"
    print_success("Métodos de SensorLogger correctos")
    
    return True

def test_src_velocity_logger():
    """Prueba el módulo src.velocity_logger"""
    print_header("TEST 5: Módulo src.velocity_logger")
    
    velocity_logger = test_import("src.velocity_logger", "(velocity logger)")
    if not velocity_logger:
        return False
    
    # Verificar clase
    assert hasattr(velocity_logger, 'VelocityLogger'), "Debe tener clase VelocityLogger"
    print_success("Clase VelocityLogger existe")
    
    # Verificar métodos
    assert hasattr(velocity_logger.VelocityLogger, '__init__'), "VelocityLogger debe tener __init__"
    assert hasattr(velocity_logger.VelocityLogger, 'start'), "VelocityLogger debe tener start"
    assert hasattr(velocity_logger.VelocityLogger, 'log'), "VelocityLogger debe tener log"
    assert hasattr(velocity_logger.VelocityLogger, 'stop'), "VelocityLogger debe tener stop"
    print_success("Métodos de VelocityLogger correctos")
    
    # Probar crear instancia (sin iniciar realmente)
    try:
        logger = velocity_logger.VelocityLogger(potential_type='linear', log_dir='logs')
        print_success("VelocityLogger se puede instanciar")
    except Exception as e:
        error_msg = f"Error instanciando VelocityLogger: {str(e)}"
        print_error(error_msg)
        errors.append(error_msg)
    
    return True

def test_utils_point_manager():
    """Prueba el módulo utils.point_manager"""
    print_header("TEST 6: Módulo utils.point_manager")
    
    # Este módulo tiene imports específicos y requiere robot conectado
    # Solo verificamos que se pueda importar sin errores de sintaxis
    try:
        pm = importlib.import_module("utils.point_manager")
        print_success("utils.point_manager se puede importar")
        
        # Verificar que tiene función main
        assert hasattr(pm, 'main'), "point_manager debe tener función main"
        print_success("Función main existe")
        
        return True
    except ImportError as e:
        # Puede fallar si falta pynput, pero eso es OK para tests básicos
        if 'pynput' in str(e):
            print_warning("pynput no disponible (esperado si no se necesita teleoperación)")
            return True
        else:
            error_msg = f"Error importando point_manager: {str(e)}"
            print_error(error_msg)
            errors.append(error_msg)
            return False
    except Exception as e:
        error_msg = f"Error inesperado en point_manager: {str(e)}"
        print_error(error_msg)
        errors.append(error_msg)
        traceback.print_exc()
        return False

def test_analysis_modules():
    """Prueba los módulos de análisis"""
    print_header("TEST 7: Módulos de análisis")
    
    # Test analyze_results
    try:
        analyze = importlib.import_module("analysis.analyze_results")
        print_success("analysis.analyze_results se puede importar")
        
        assert hasattr(analyze, 'load_csv'), "Debe tener función load_csv"
        assert hasattr(analyze, 'analyze_trajectory'), "Debe tener función analyze_trajectory"
        assert hasattr(analyze, 'main'), "Debe tener función main"
        print_success("Funciones de analyze_results correctas")
    except Exception as e:
        error_msg = f"Error en analyze_results: {str(e)}"
        print_error(error_msg)
        errors.append(error_msg)
        traceback.print_exc()
    
    # Test visualize_safety
    try:
        visualize = importlib.import_module("analysis.visualize_safety")
        print_success("analysis.visualize_safety se puede importar")
        
        assert hasattr(visualize, 'plot_ir_vs_vmax'), "Debe tener función plot_ir_vs_vmax"
        assert hasattr(visualize, 'plot_ir_vs_distance'), "Debe tener función plot_ir_vs_distance"
        assert hasattr(visualize, 'plot_comparison_table'), "Debe tener función plot_comparison_table"
        assert hasattr(visualize, 'main'), "Debe tener función main"
        print_success("Funciones de visualize_safety correctas")
    except Exception as e:
        error_msg = f"Error en visualize_safety: {str(e)}"
        print_error(error_msg)
        errors.append(error_msg)
        traceback.print_exc()
    
    return True

def test_main_scripts():
    """Prueba que los scripts principales se puedan importar como módulos"""
    print_header("TEST 8: Scripts principales")
    
    # Los scripts principales pueden tener código que se ejecuta al importar
    # Por eso solo verificamos sintaxis básica
    scripts = ['PRM01_P01', 'PRM01_P02']
    
    for script_name in scripts:
        try:
            # Intentar compilar el archivo para verificar sintaxis
            script_path = Path(f"{script_name}.py")
            if script_path.exists():
                with open(script_path, 'r', encoding='utf-8') as f:
                    code = f.read()
                compile(code, script_path, 'exec')
                print_success(f"{script_name}.py tiene sintaxis válida")
            else:
                error_msg = f"Script {script_name}.py no encontrado"
                print_error(error_msg)
                errors.append(error_msg)
        except SyntaxError as e:
            error_msg = f"Error de sintaxis en {script_name}.py: {str(e)}"
            print_error(error_msg)
            errors.append(error_msg)
        except Exception as e:
            error_msg = f"Error verificando {script_name}.py: {str(e)}"
            print_error(error_msg)
            errors.append(error_msg)
    
    return True

def test_files_structure():
    """Verifica que los archivos y directorios necesarios existan"""
    print_header("TEST 9: Estructura de archivos")
    
    required_files = [
        'src/__init__.py',
        'src/config.py',
        'src/potential_fields.py',
        'src/safety.py',
        'src/sensor_logger.py',
        'src/velocity_logger.py',
        'utils/point_manager.py',
        'analysis/__init__.py',
        'analysis/analyze_results.py',
        'analysis/visualize_safety.py',
        'PRM01_P01.py',
        'PRM01_P02.py',
        'README.md'
    ]
    
    all_exist = True
    for filepath in required_files:
        if not test_file_exists(filepath):
            all_exist = False
    
    # Verificar directorios
    required_dirs = ['src', 'utils', 'analysis', 'data', 'logs', 'images']
    for dirpath in required_dirs:
        path = Path(dirpath)
        if path.exists() and path.is_dir():
            print_success(f"Directorio existe: {dirpath}")
        else:
            print_warning(f"Directorio no existe: {dirpath} (puede crearse automáticamente)")
    
    # Verificar points.json
    if test_file_exists('data/points.json'):
        data = test_json_file('data/points.json')
        if data:
            if 'q_i' in data and 'q_f' in data:
                print_success("points.json tiene estructura correcta (q_i y q_f)")
            else:
                print_warning("points.json existe pero puede no tener estructura completa")
    
    return all_exist

def test_cross_module_imports():
    """Prueba imports cruzados entre módulos"""
    print_header("TEST 10: Imports cruzados")
    
    # Verificar que potential_fields puede importar config
    try:
        from src import potential_fields
        from src import config as config_module
        
        # Verificar que potential_fields use config correctamente
        assert hasattr(potential_fields, 'attractive_wheel_speeds'), "potential_fields debe tener attractive_wheel_speeds"
        
        # Verificar que safety puede importar config
        from src import safety
        assert hasattr(safety, 'saturate_wheel_speeds'), "safety debe tener saturate_wheel_speeds"
        
        print_success("Imports cruzados funcionan correctamente")
        return True
    except Exception as e:
        error_msg = f"Error en imports cruzados: {str(e)}"
        print_error(error_msg)
        errors.append(error_msg)
        traceback.print_exc()
        return False

# ============ FUNCIÓN PRINCIPAL ============

def main():
    """Ejecuta todas las pruebas"""
    print_header("INICIANDO PRUEBAS COMPLETAS DEL PROYECTO")
    
    print_info(f"Python version: {sys.version}")
    print_info(f"Directorio de trabajo: {Path.cwd()}\n")
    
    # Ejecutar todas las pruebas
    tests = [
        ("Estructura de archivos", test_files_structure),
        ("src.config", test_src_config),
        ("src.potential_fields", test_src_potential_fields),
        ("src.safety", test_src_safety),
        ("src.sensor_logger", test_src_sensor_logger),
        ("src.velocity_logger", test_src_velocity_logger),
        ("utils.point_manager", test_utils_point_manager),
        ("Módulos de análisis", test_analysis_modules),
        ("Scripts principales", test_main_scripts),
        ("Imports cruzados", test_cross_module_imports),
    ]
    
    results = []
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            error_msg = f"Error ejecutando {test_name}: {str(e)}"
            print_error(error_msg)
            errors.append(error_msg)
            traceback.print_exc()
            results.append((test_name, False))
    
    # Resumen final
    print_header("RESUMEN DE PRUEBAS")
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    print(f"\n{Colors.BOLD}Pruebas pasadas: {passed}/{total}{Colors.RESET}\n")
    
    for test_name, result in results:
        if result:
            print_success(f"{test_name}")
        else:
            print_error(f"{test_name}")
    
    if errors:
        print(f"\n{Colors.BOLD}{Colors.RED}ERRORES ENCONTRADOS: {len(errors)}{Colors.RESET}\n")
        for i, error in enumerate(errors, 1):
            print(f"{i}. {error}")
    
    if warnings:
        print(f"\n{Colors.BOLD}{Colors.YELLOW}ADVERTENCIAS: {len(warnings)}{Colors.RESET}\n")
        for i, warning in enumerate(warnings, 1):
            print(f"{i}. {warning}")
    
    # Resultado final
    print_header("RESULTADO FINAL")
    
    if len(errors) == 0:
        print(f"\n{Colors.GREEN}{Colors.BOLD}[SUCCESS] TODAS LAS PRUEBAS PASARON CORRECTAMENTE{Colors.RESET}\n")
        return 0
    else:
        print(f"\n{Colors.RED}{Colors.BOLD}[FAILED] SE ENCONTRARON {len(errors)} ERROR(ES){Colors.RESET}\n")
        return 1

if __name__ == "__main__":
    sys.exit(main())

