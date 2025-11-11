# Calibración de Sensores IR - Robot Create3

## Descripción General

Esta documentación contiene los datos de calibración para el sistema de evasión de obstáculos del robot Create3. Los datos fueron recopilados con obstáculos posicionados a **5cm del borde del robot** para establecer valores de referencia precisos.

> **Importante**: El sistema de evasión de obstáculos debe ser confiable y no fallar. Es crucial comprender los valores obtenidos y las variaciones entre sensores.

## Configuración de Sensores IR

El robot Create3 cuenta con **7 sensores de proximidad IR** numerados del 0 al 6, distribuidos alrededor del perímetro del robot.

### Distribución de Sensores
- **Sensores frontales**: Mayor sensibilidad para detección frontal
- **Sensores laterales**: Sensibilidad diferenciada entre izquierda y derecha
- **Variaciones**: Cada sensor tiene características únicas de respuesta

---

## Datos de Calibración

### 1. Obstáculo Frontal (Pared a 90°)

**Configuración**: Pared perpendicular al frente del robot

#### Lectura 1
```
Proximidad IR:
  0: 6      ← Sensor frontal izquierdo
  1: 20     ← Sensor frontal centro-izquierdo  
  2: 271    ← Sensor frontal centro
  3: 1044   ← Sensor frontal centro-derecho
  4: 895    ← Sensor frontal derecho
  5: 14     ← Sensor lateral derecho
  6: 6      ← Sensor lateral izquierdo
```

#### Lectura 2
```
Proximidad IR:
  0: 6      ← Sensor frontal izquierdo
  1: 22     ← Sensor frontal centro-izquierdo
  2: 268    ← Sensor frontal centro
  3: 1046   ← Sensor frontal centro-derecho
  4: 898    ← Sensor frontal derecho
  5: 15     ← Sensor lateral derecho
  6: 7      ← Sensor lateral izquierdo
```

**Análisis**: Los sensores 3 y 4 muestran los valores más altos (1044-1046 y 895-898), indicando detección frontal directa.

---

### 2. Obstáculo a 45° del Lado Derecho

**Configuración**: Obstáculo diagonal desde el lado derecho

#### Lectura 1
```
Proximidad IR:
  0: 11     ← Sensor frontal izquierdo
  1: 9      ← Sensor frontal centro-izquierdo
  2: 24     ← Sensor frontal centro
  3: 14     ← Sensor frontal centro-derecho
  4: 39     ← Sensor frontal derecho
  5: 669    ← Sensor lateral derecho (ALTO)
  6: 660    ← Sensor lateral izquierdo (ALTO)
```

#### Lectura 2
```
Proximidad IR:
  0: 4      ← Sensor frontal izquierdo
  1: 11     ← Sensor frontal centro-izquierdo
  2: 22     ← Sensor frontal centro
  3: 27     ← Sensor frontal centro-derecho
  4: 34     ← Sensor frontal derecho
  5: 676    ← Sensor lateral derecho (ALTO)
  6: 660    ← Sensor lateral izquierdo (ALTO)
```

**Datos Adicionales**:
- Sensores de Caída (cliff): Todos en False
- Batería: 100% (16362 mV)

**Análisis**: Los sensores 5 y 6 muestran valores altos (669-676 y 660), confirmando detección lateral derecha.

---

### 3. Obstáculo a 45° del Lado Izquierdo

**Configuración**: Obstáculo diagonal desde el lado izquierdo

#### Lectura 1
```
Proximidad IR:
  0: 774    ← Sensor frontal izquierdo (ALTO)
  1: 1121   ← Sensor frontal centro-izquierdo (MUY ALTO)
  2: 291    ← Sensor frontal centro
  3: 19     ← Sensor frontal centro-derecho
  4: 14     ← Sensor frontal derecho
  5: 9      ← Sensor lateral derecho
  6: 4      ← Sensor lateral izquierdo
```

#### Lectura 2
```
Proximidad IR:
  0: 771    ← Sensor frontal izquierdo (ALTO)
  1: 1123   ← Sensor frontal centro-izquierdo (MUY ALTO)
  2: 288    ← Sensor frontal centro
  3: 21     ← Sensor frontal centro-derecho
  4: 12     ← Sensor frontal derecho
  5: 7      ← Sensor lateral derecho
  6: 6      ← Sensor lateral izquierdo
```

**Datos Adicionales**:
- Posición: x=35.00 cm, y=-4.80 cm, θ=155.6°
- Acelerómetro: X=0, Y=10, Z=0
- Bumpers: Izq=False, Der=False
- Botones: 1=False, 2=False

**Análisis**: Los sensores 0 y 1 muestran valores muy altos (774-771 y 1121-1123), indicando detección lateral izquierda.

---

### 4. Obstáculo Perpendicular al Sensor 0

**Configuración**: Obstáculo directamente frente al sensor frontal izquierdo

#### Lectura 1
```
Proximidad IR:
  0: 1382   ← Sensor frontal izquierdo 
  1: 50     ← Sensor frontal centro-izquierdo
  2: 16     ← Sensor frontal centro
  3: 16     ← Sensor frontal centro-derecho
  4: 12     ← Sensor frontal derecho
  5: 8      ← Sensor lateral derecho
  6: 0      ← Sensor lateral izquierdo
```

#### Lectura 2
```
Proximidad IR:
  0: 1386   ← Sensor frontal izquierdo 
  1: 52     ← Sensor frontal centro-izquierdo
  2: 16     ← Sensor frontal centro
  3: 14     ← Sensor frontal centro-derecho
  4: 12     ← Sensor frontal derecho
  5: 7      ← Sensor lateral derecho
  6: 12     ← Sensor lateral izquierdo
```

**Análisis**: El sensor 0 alcanza valores máximos (1382-1386), confirmando detección directa.

---

### 5. Obstáculo Perpendicular al Sensor 6

**Configuración**: Obstáculo directamente frente al sensor lateral izquierdo

#### Lectura 1
```
Proximidad IR:
  0: 6      ← Sensor frontal izquierdo
  1: 9      ← Sensor frontal centro-izquierdo
  2: 16     ← Sensor frontal centro
  3: 14     ← Sensor frontal centro-derecho
  4: 9      ← Sensor frontal derecho
  5: 4      ← Sensor lateral derecho
  6: 902    ← Sensor lateral izquierdo (ALTO)
```

#### Lectura 2
```
Proximidad IR:
  0: 7      ← Sensor frontal izquierdo
  1: 1      ← Sensor frontal centro-izquierdo
  2: 17     ← Sensor frontal centro
  3: 12     ← Sensor frontal centro-derecho
  4: 10     ← Sensor frontal derecho
  5: 2      ← Sensor lateral derecho
  6: 900    ← Sensor lateral izquierdo (ALTO)
```

**Análisis**: El sensor 6 muestra valores altos (902-900), confirmando detección lateral izquierda.

---

## Resumen de Patrones de Detección

### Rangos de Valores por Tipo de Detección

| Tipo de Detección | Rango de Valores | Sensores Involucrados |
|-------------------|------------------|----------------------|
| **Sin obstáculo** | 0-50 | Todos los sensores |
| **Detección leve** | 50-300 | Sensores cercanos |
| **Detección moderada** | 300-700 | Sensores principales |
| **Detección fuerte** | 700-1000 | Sensores directos |
| **Detección máxima** | 1000+ | Sensor perpendicular |

### Sensores Más Sensibles

1. **Sensor 0** (Frontal izquierdo): Máximo 1386
2. **Sensor 1** (Frontal centro-izquierdo): Máximo 1123
3. **Sensor 3** (Frontal centro-derecho): Máximo 1046
4. **Sensor 4** (Frontal derecho): Máximo 898
5. **Sensor 6** (Lateral izquierdo): Máximo 902
6. **Sensor 5** (Lateral derecho): Máximo 676

### Recomendaciones para Implementación

1. **Umbral de Evasión**: Establecer umbral mínimo de 300 para activar evasión
2. **Umbral Crítico**: Valores > 1000 requieren evasión inmediata
3. **Variaciones**: Considerar ±50 unidades como margen de error
4. **Calibración Continua**: Monitorear variaciones durante operación

---

## Notas Importantes

- **Distancia de referencia**: Todos los obstáculos a 5cm del borde del robot
- **Variaciones**: Pequeñas diferencias entre lecturas son normales
- **Sensibilidad**: Los sensores frontales tienen mayor sensibilidad que los laterales

---

