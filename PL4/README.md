# Práctica 4 - Navegación con Campos de Potencial

**Autores:** Yago Ramos - Alan Salazar  
**Fecha:** 28 de octubre de 2025  
**Institución:** UIE - Robots Autónomos  
**Robot:** iRobot Create 3

## 🆕 Actualización - Sistema de Seguridad Mejorado (v2.0)

### Cambios Importantes

Se ha implementado un **sistema de umbrales escalonados** para control robusto de velocidad ante obstáculos:

- **4 niveles de seguridad** (vs 2 anteriores)
- **Límites dinámicos de velocidad** según proximidad
- **Reacción gradual** para evitar colisiones
- **Basado en calibración real** del robot a 5cm

Ver detalles completos en: [`SAFETY_THRESHOLDS.md`](SAFETY_THRESHOLDS.md)

### Umbrales Actualizados

| Nivel | IR Umbral | V_max | Distancia Est. |
|-------|-----------|-------|----------------|
| 🚨 EMERGENCIA | ≥800 | 0 cm/s | <5 cm |
| 🔴 CRÍTICO | ≥400 | 10 cm/s | 5-10 cm |
| ⚠️ ADVERTENCIA | ≥200 | 20 cm/s | 10-20 cm |
| ⚡ PRECAUCIÓN | ≥100 | 30 cm/s | 20-40 cm |
| ✅ LIBRE | <100 | 48 cm/s | >40 cm |

## Descripción

Implementación de navegación autónoma mediante campos de potencial combinados (atractivo + repulsivo) para el robot Create 3. El sistema permite al robot navegar desde un punto inicial hasta un objetivo mientras evita obstáculos detectados por sensores IR, con **control inteligente de velocidad** que garantiza tiempo suficiente de reacción.

## Estructura del Proyecto

```
PL4/
├── config.py              # Parámetros de configuración
├── potential_fields.py    # Funciones de campos de potencial
├── safety.py             # Sistema de seguridad
├── point_manager.py      # Gestión de puntos de navegación
├── sensor_logger.py      # Registro de sensores
├── velocity_logger.py    # Registro de velocidades
├── PRM01_P01.py          # Script Parte 01 (solo atractivo)
├── PRM01_P02.py          # Script Parte 02 (combinado)
├── analyze_results.py    # Análisis de resultados
├── points.json           # Puntos de navegación
└── logs/                 # Archivos CSV de telemetría
```

## Parte 01 - Campo de Potencial Atractivo

### Objetivo
Implementar navegación usando únicamente el campo de potencial atractivo que lleva al robot hacia la meta.

### Funciones de Potencial Implementadas

1. **Lineal:** F = k * d
   - Proporcional a la distancia
   - Velocidad constante independiente de la distancia

2. **Cuadrático:** F = k * d²
   - Crece con el cuadrado de la distancia
   - Mayor aceleración al estar lejos

3. **Cónico:** F = k * min(d, d_sat)
   - Saturación en distancia máxima
   - Velocidad constante cuando d > 100 cm

4. **Exponencial:** F = k * (1 - e^(-d/λ))
   - Convergencia asintótica
   - Aceleración suave al inicio
```

## Parte 02 - Campo de Potencial Combinado

### Objetivo
Añadir campo de potencial repulsivo para evasión de obstáculos detectados por sensores IR, combinándolo con el potencial atractivo.

### Metodología

El sistema utiliza una estrategia híbrida que combina:

1. **Velocidad base del potencial atractivo**
   - Determina la velocidad de avance hacia la meta
   - Independiente de los obstáculos

2. **Dirección ajustada por fuerzas repulsivas**
   - Los obstáculos modifican el ángulo de movimiento
   - Combinación ponderada según proximidad del obstáculo

3. **Reducción de velocidad por proximidad**
   - Slowdown factor proporcional a la fuerza repulsiva
   - Mantiene velocidad mínima para ejecutar maniobras

### Fórmulas Implementadas

**Fuerza Repulsiva:**
```
F_rep = k_rep * (I/1000) * (1/d - 1/d_inf)

donde:
  k_rep = 500.0 (ganancia repulsiva)
  I = valor del sensor IR (0-4095)
  d = distancia estimada al obstáculo
  d_inf = 30.0 cm (distancia de influencia)
```

**Estimación de Distancia (modelo físico I ∝ 1/d²):**
```
d = 5.0 * sqrt(1000 / I)

Calibración basada en:
  - I = 1000 corresponde a d = 5 cm
  - Rango válido: 5-40 cm
```

**Combinación de Ángulos:**
```
w_rep = min(|F_rep| / 5.0, 0.9)
w_att = 1 - w_rep

θ_combined = atan2(
  w_att * sin(θ_goal) + w_rep * sin(θ_rep),
  w_att * cos(θ_goal) + w_rep * cos(θ_rep)
)
```

### Parámetros de Configuración

```python
# Potencial Atractivo
K_LINEAR = 0.25
K_QUADRATIC = 0.01
K_CONIC = 0.15
K_EXPONENTIAL = 2.5
K_ANGULAR = 1.2

# Potencial Repulsivo
K_REPULSIVE = 500.0
D_INFLUENCE = 30.0  # cm
D_SAFE = 8.0        # cm

# Control de Velocidad
V_MAX_CM_S = 48.0
V_MIN_CM_S = 0.0
CONTROL_DT = 0.05   # 20 Hz

# Seguridad - Sistema Escalonado (v2.0)
IR_THRESHOLD_EMERGENCY = 800   # PARAR: <5cm
IR_THRESHOLD_CRITICAL = 400    # V_max=10cm/s: 5-10cm
IR_THRESHOLD_WARNING = 200     # V_max=20cm/s: 10-20cm
IR_THRESHOLD_CAUTION = 100     # V_max=30cm/s: 20-40cm
```

### Uso

```bash
# Navegación básica con potencial cónico
python PRM01_P02.py --potential conic

# Ajustar parámetros repulsivos
python PRM01_P02.py --potential conic --k-rep 500 --d-influence 30

# Modo debug
python PRM01_P02.py --potential conic --debug
```

## 🧪 Scripts de Prueba y Análisis

### Test del Sistema de Seguridad

```bash
# Ver demostración de umbrales
python test_safety_thresholds.py

# Test rápido de navegación con logging
python quick_test.py

# Comparar logs antiguos vs nuevos
python compare_logs.py
```

### Análisis de Resultados

```bash
# Generar gráficas de telemetría
python analyze_results.py logs/velocities_conic_combined_YYYYMMDD_HHMMSS.csv
```

Genera:
- Trayectoria en el plano XY
- Evolución de velocidades con niveles de seguridad
- Fuerzas atractivas y repulsivas
- Detección de obstáculos
- Distribución de niveles de seguridad

## Calibración de Sensores IR

Los sensores IR están calibrados según datos experimentales a 5 cm:

| Sensor | Ángulo | Posición | Valor típico a 5cm |
|--------|--------|----------|-------------------|
| 0 | +65.3° | Lateral izquierdo | 774-1386 |
| 1 | +38.0° | Intermedio izq | 1121-1123 |
| 2 | +20.0° | Frontal izq | 268-291 |
| 3 | -3.0° | Centro | 1044-1046 |
| 4 | -14.25° | Frontal der | 895-898 |
| 5 | -34.0° | Intermedio der | 669-676 |
| 6 | -65.3° | Lateral derecho | 900-902 |

Valores de referencia:
- Perpendicular: 1300-1400
- Frontal directo: 900-1100
- Ángulo 45°: 600-700
- Ángulo oblicuo: 250-300

## Comportamiento del Sistema

### Sin Obstáculos
- Navegación directa hacia la meta
- Velocidad determinada por función de potencial
- Corrección angular suave

### Con Obstáculos
- Detección mediante sensores IR
- Reducción de velocidad proporcional a proximidad
- Giro para evadir según posición del obstáculo
- Velocidad mínima de 1 cm/s para maniobras efectivas

### Criterios de Detención
- Llegada a meta: distancia < 3 cm
- Colisiones físicas: activación de bumpers
- Máximo de colisiones: 3 intentos

## Solución de Problemas

### El robot no evade obstáculos
- Verificar K_REPULSIVE (debe ser ~500)
- Comprobar calibración de sensores IR
- Revisar D_INFLUENCE (30 cm recomendado)

### El robot oscila sin avanzar
- Aumentar velocidad mínima de evasión
- Reducir K_ANGULAR si gira demasiado
- Verificar que angle_factor_min >= 0.1

### Colisiones frecuentes
- Aumentar K_REPULSIVE
- Reducir V_MAX_CM_S
- Aumentar D_INFLUENCE

## Archivos de Salida

Los logs se guardan en `logs/` con timestamp:

```
velocities_[tipo]_[YYYYMMDD]_[HHMMSS].csv
```

Contiene:
- Timestamp
- Posición (x, y, θ)
- Distancia a meta
- Velocidades (v_left, v_right, v_linear, ω)
- Fuerzas (atractiva, repulsiva, total)
- Número de obstáculos detectados
- Tipo de potencial

## Notas de Implementación

### Convención de Ángulos
- Marco global: θ = 0° apunta a +X (este)
- Crecimiento antihorario (convención atan2)
- Ángulos de sensores: desde el frente del robot
  - Positivos: hacia la izquierda
  - Negativos: hacia la derecha

### Estrategia de Evasión
1. Calcular velocidad base del potencial atractivo
2. Detectar obstáculos y calcular fuerzas repulsivas
3. Combinar ángulos de atracción y repulsión
4. Aplicar reducción de velocidad según proximidad
5. Mantener velocidad mínima para ejecutar maniobras

### Limitaciones
- Posibles mínimos locales en configuraciones de obstáculos complejas
- Alcance limitado de sensores IR (~40 cm efectivo)
- Comportamiento subóptimo en pasillos estrechos

## Referencias

- iRobot Create 3 Documentation
- Khatib, O. (1986). Real-time obstacle avoidance for manipulators and mobile robots
- Calibración experimental de sensores IR (ver `Calibracion/CALIBRACION_create3.md`)
