# Sistema de Umbrales de Seguridad Escalonados

## Fecha de actualización: 28 de octubre de 2025

## Problema Identificado

El robot alcanzaba velocidades altas (hasta 48 cm/s) en trayectos largos, y los umbrales originales de detección de obstáculos eran demasiado bajos:
- **IR_THRESHOLD_SLOW = 150**: Muy bajo, detectaba obstáculos cuando ya estaba muy cerca
- **IR_THRESHOLD_STOP = 300**: Insuficiente tiempo de reacción a alta velocidad

Esto causaba **colisiones frecuentes** porque el robot no tenía suficiente distancia/tiempo para frenar o evadir.

## Solución Implementada

### Sistema de 4 Niveles de Seguridad

Basado en los datos de **calibración real** (obstáculos a 5cm):

| Nivel | Umbral IR | V_max Permitida | Distancia Estimada | Estado |
|-------|-----------|-----------------|-------------------|---------|
| **🚨 EMERGENCIA** | ≥ 800 | 0 cm/s (PARAR) | < 5 cm | Obstáculo muy cerca, perpendicular |
| **🔴 CRÍTICO** | ≥ 400 | 10 cm/s | ~5-10 cm | Muy cerca, velocidad mínima |
| **⚠️ ADVERTENCIA** | ≥ 200 | 20 cm/s | ~10-20 cm | Distancia media, velocidad reducida |
| **⚡ PRECAUCIÓN** | ≥ 100 | 30 cm/s | ~20-40 cm | Distancia segura, velocidad limitada |
| **✅ LIBRE** | < 100 | 48 cm/s | > 40 cm | Sin obstáculos, velocidad normal |

### Referencia de Calibración

Valores típicos según configuración (obstáculo a 5cm):
- **Frontal perpendicular**: 900-1400
- **Frontal directo**: 600-1100  
- **Ángulo 45°**: 400-700
- **Ángulo oblicuo**: 200-400

## Implementación Técnica

### 1. Configuración (`config.py`)

```python
# Umbrales escalonados
IR_THRESHOLD_EMERGENCY = 800    # PARAR
IR_THRESHOLD_CRITICAL = 400     # V_max = 10 cm/s
IR_THRESHOLD_WARNING = 200      # V_max = 20 cm/s
IR_THRESHOLD_CAUTION = 100      # V_max = 30 cm/s
IR_THRESHOLD_DETECT = 50        # Detección mínima

# Límites de velocidad dinámicos
V_MAX_EMERGENCY = 0.0
V_MAX_CRITICAL = 10.0
V_MAX_WARNING = 20.0
V_MAX_CAUTION = 30.0
```

### 2. Control Predictivo (`potential_fields.py`)

El sistema analiza los **sensores frontales críticos** (1, 2, 3, 4) en cada iteración:

```python
max_ir_front = max(ir_sensors[1], ir_sensors[2], ir_sensors[3], ir_sensors[4])

# Determinar v_max dinámico según nivel
if max_ir_front >= IR_THRESHOLD_EMERGENCY:
    v_max_allowed = 0.0  # PARAR
elif max_ir_front >= IR_THRESHOLD_CRITICAL:
    v_max_allowed = 10.0
# ... etc
```

La velocidad calculada **nunca excede** `v_max_allowed`, garantizando control seguro.

### 3. Monitoreo en Tiempo Real (`sensor_logger.py`)

Muestra el nivel de seguridad actual:

```
IR: [0]=   5 [1]= 412 [2]= 385 [3]= 401 [4]= 367 [5]=  38 [6]=   4
   Max frontal:  412  🔴 CRÍTICO  (v≤10cm/s)
```

## Ventajas del Sistema

1. **Reacción Gradual**: El robot reduce velocidad progresivamente, no de golpe
2. **Tiempo de Frenado**: Máximo 30 cm/s cuando detecta algo a 20+ cm
3. **Sin Falsos Positivos**: Umbral mínimo de 100 evita reacciones a ruido
4. **Evasión Efectiva**: Permite maniobras a velocidad controlada
5. **Visible en Logs**: Cada nivel se registra en CSV para análisis

## Comportamiento Esperado

### Escenario 1: Trayecto Libre
- Robot acelera hasta 48 cm/s
- Sensores frontales < 100
- Avance rápido y eficiente

### Escenario 2: Obstáculo Lejano
- Sensor detecta IR = 150
- Nivel: **⚠️ ADVERTENCIA**
- V_max → 20 cm/s
- Robot reduce velocidad gradualmente, tiene tiempo para evadir

### Escenario 3: Obstáculo Cercano
- Sensor detecta IR = 450
- Nivel: **🔴 CRÍTICO**
- V_max → 10 cm/s
- Robot casi para, maniobra lenta de evasión

### Escenario 4: Obstáculo Muy Cercano
- Sensor detecta IR = 850
- Nivel: **🚨 EMERGENCIA**
- V_max → 0 cm/s
- Robot PARA completamente (prevención de colisión)

## Ajustes Futuros

Si se observan **colisiones persistentes**:
- ↑ Aumentar umbrales (ej: CAUTION a 120)
- ↓ Reducir velocidades máximas (ej: WARNING a 15 cm/s)

Si el robot es **demasiado conservador**:
- ↓ Reducir umbrales (ej: CAUTION a 80)
- ↑ Aumentar velocidades permitidas (ej: WARNING a 25 cm/s)

## Testing Recomendado

1. **Test en recta larga**: Verificar que alcanza 48 cm/s sin obstáculos
2. **Test con obstáculo frontal**: Confirmar reducción a 20 cm/s antes de colisión
3. **Test de evasión lateral**: Observar maniobras a velocidad controlada
4. **Análisis de logs**: Revisar `velocities_*.csv` para ver transiciones de niveles

---

**Nota**: Este sistema es conservador por diseño. Es preferible un robot lento y seguro que uno rápido con colisiones frecuentes.
