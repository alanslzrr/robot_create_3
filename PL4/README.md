# Práctica 4 - Navegación con Campos de Potencial

**Autores:** Alan Ariel Salazar, Yago Ramos Sánchez  
**Fecha de Finalización:** 6 de noviembre de 2025  
**Institución:** Universidad Intercontinental de la Empresa (UIE)  
**Asignatura:** Robots Autónomos  
**Profesor:** Eladio Dapena  
**Robot:** iRobot Create 3

---

## 📑 Navegación del Documento

- [**Introducción**](#introducción) — Visión general del proyecto
- [**Estructura del Proyecto**](#estructura-del-proyecto) — Organización de archivos y módulos
- [**Parte 01**](#parte-01---campo-de-potencial-atractivo) — Navegación con potencial atractivo únicamente
- [**Parte 02**](#parte-02---campo-de-potencial-combinado) — Navegación con evitación de obstáculos
- [**Funciones de Potencial Atractivo**](#funciones-de-potencial-atractivo) — Cuatro variantes implementadas con fórmulas
- [**Sistema Repulsivo Completo**](#sistema-repulsivo-completo) — De sensores IR a fuerzas repulsivas
- [**Combinación de Fuerzas**](#combinación-de-fuerzas) — Integración vectorial atractiva y repulsiva
- [**Control de Velocidad**](#control-de-velocidad-dinámico) — Sistema adaptativo con cinco niveles
- [**Cinemática del Robot**](#cinemática-diferencial) — Conversión a velocidades de rueda
- [**Detección de Gaps**](#detección-de-gaps-navegables) — Espacios navegables entre obstáculos
- [**Características Avanzadas**](#características-avanzadas) — Escape de trampas y transformaciones
- [**Configuración y Uso**](#configuración-y-uso) — Cómo ejecutar el sistema
- [**Análisis de Resultados**](#salida-y-análisis) — Herramientas de evaluación y visualización
- [**Parámetros del Sistema**](#parámetros-principales) — Valores calibrados y constantes
- [**Conclusiones**](#resultados-y-conclusiones) — Resumen de logros

---

## Introducción

Este proyecto implementa un sistema completo de navegación autónoma para el robot iRobot Create 3 utilizando campos de potencial. El objetivo principal es lograr que el robot navegue desde una posición inicial hasta una posición final, primero usando únicamente un campo de potencial atractivo, y posteriormente combinando este campo atractivo con un campo repulsivo para evitar obstáculos detectados por los sensores infrarrojos.

El proyecto se desarrolló en dos partes principales, cada una implementada en un script separado que permite probar diferentes funciones de potencial y analizar su comportamiento comparativo. La estructura del código está organizada en módulos reutilizables que facilitan el mantenimiento y la extensión del sistema.

## Estructura del Proyecto

El proyecto está organizado en varias carpetas y archivos principales que cumplen funciones específicas:

```
PL4/
├── PRM01_P01.py          # Script principal Parte 01 (potencial atractivo)
├── PRM01_P02.py          # Script principal Parte 02 (potencial combinado)
├── src/                  # Módulos principales del sistema
│   ├── config.py         # Configuración centralizada de parámetros
│   ├── potential_fields.py  # Implementación de funciones de potencial
│   ├── safety.py         # Sistema de seguridad y detección de obstáculos
│   ├── sensor_logger.py  # Monitoreo de sensores en tiempo real
│   └── velocity_logger.py # Registro de datos en CSV
├── utils/                # Herramientas auxiliares
│   └── point_manager.py  # Configuración de puntos de navegación
├── analysis/             # Scripts de análisis y visualización
│   ├── analyze_results.py    # Análisis comparativo de resultados CSV
│   └── visualize_safety.py  # Generación de gráficos del sistema de seguridad
├── data/                 # Archivos de datos
│   └── points.json       # Puntos inicial y final de navegación
├── images/               # Imágenes generadas por scripts de visualización
└── logs/                 # Archivos CSV con datos de telemetría
```

## Parte 01 - Campo de Potencial Atractivo

En `PRM01_P01.py` implementamos la navegación básica utilizando únicamente un campo de potencial atractivo. Este campo genera una fuerza que atrae al robot hacia la posición objetivo, calculando en cada iteración las velocidades de las ruedas necesarias para avanzar en esa dirección.

El script integra varios módulos del sistema:
- **potential_fields.py**: Contiene las cuatro funciones de potencial implementadas
- **safety.py**: Aplica límites de seguridad a las velocidades calculadas
- **sensor_logger.py**: Monitorea el estado de los sensores durante la ejecución
- **velocity_logger.py**: Registra todos los datos de navegación en archivos CSV

Para ejecutar este script necesitamos primero configurar los puntos de navegación usando `point_manager.py`, que genera el archivo `points.json` en la carpeta `data`. Este archivo contiene:
- **q_i**: Posición inicial con coordenadas $(x, y)$ en centímetros y orientación $\theta$ en grados (donde $0°$ apunta hacia el eje positivo $X$, y los ángulos crecen en sentido antihorario)
- **q_f**: Posición final con coordenadas $(x, y)$ del objetivo

## Parte 02 - Campo de Potencial Combinado

En `PRM01_P02.py` extendimos la funcionalidad anterior para incluir un campo de potencial repulsivo que evita obstáculos. Esta implementación combina el campo atractivo hacia la meta con fuerzas repulsivas calculadas a partir de las lecturas de los siete sensores infrarrojos del robot.

La diferencia principal es que ahora utilizamos `combined_potential_speeds()` en lugar de `attractive_wheel_speeds()`. Esta función:
- Lee continuamente los siete sensores infrarrojos del robot
- Estima la posición de los obstáculos basándose en un modelo físico mejorado
- Calcula fuerzas repulsivas basadas en el concepto de *clearance* (distancia libre después del radio del robot)
- Detecta espacios navegables (gaps) entre obstáculos
- Combina vectorialmente las fuerzas atractivas y repulsivas

El resultado es una navegación que se ajusta dinámicamente para evitar colisiones mientras mantiene el objetivo de llegar a la meta.

## Funciones de Potencial Atractivo

Implementamos cuatro funciones de potencial atractivo diferentes, cada una con características matemáticas particulares que afectan el comportamiento del robot.

### Función Lineal

La función lineal genera una fuerza directamente proporcional a la distancia al objetivo:

$$F_{atractiva} = k_{lin} \cdot d$$

**Parámetros:**
- $k_{lin} = 0.25$ (definido en `config.K_LINEAR`)
- $d$ = distancia al objetivo en cm

**Implementación:**
```python
v_linear = k_lin * distance
```

El comportamiento es predecible y directo, manteniendo una velocidad aproximadamente constante durante todo el trayecto una vez que se alcanza la velocidad máxima.

### Función Cuadrática

En esta función la fuerza crece con el cuadrado de la distancia:

$$F_{atractiva} = k_{quad} \cdot \frac{d^2}{10}$$

**Parámetros:**
- $k_{quad} = 0.05$ (definido en `config.K_QUADRATIC`)
- Factor de normalización = 10 (para ajustar escala)

**Implementación:**
```python
v_linear = k_quad * (distance ** 2) / 10.0
```

El robot acelera más agresivamente cuando está lejos del objetivo y desacelera de forma más suave cuando se acerca.

### Función Cónica

Esta función incluye una saturación a una distancia máxima determinada:

$$F_{atractiva} = k_{conic} \cdot \min(d, d_{sat}) \cdot 2$$

**Parámetros:**
- $k_{conic} = 0.15$ (definido en `config.K_CONIC`)
- $d_{sat} = 100$ cm (distancia de saturación)
- Factor de amplificación = 2

**Implementación:**
```python
d_sat = 100.0
v_linear = k_conic * min(distance, d_sat) * 2.0
```

Cuando el robot está más lejos que la distancia de saturación, la velocidad se mantiene constante, y solo cuando se acerca comienza a reducir la velocidad. Útil para navegación en espacios grandes.

### Función Exponencial

La función exponencial presenta una convergencia asintótica:

$$F_{atractiva} = k_{exp} \cdot (1 - e^{-d/\lambda}) \cdot 20$$

**Parámetros:**
- $k_{exp} = 2.5$ (definido en `config.K_EXPONENTIAL`)
- $\lambda = 50$ cm (parámetro de escala)
- Factor de amplificación = 20

**Implementación:**
```python
lambda_param = 50.0
v_linear = k_exp * (1.0 - math.exp(-distance / lambda_param)) * 20.0
```

Acelera rápidamente al inicio pero desacelera de forma muy suave conforme se acerca al objetivo. Útil cuando queremos un comportamiento más suave cerca de la meta.

## Sistema Repulsivo Completo

El campo repulsivo se construye a partir de las lecturas de los sensores infrarrojos mediante un proceso de varias etapas que transforma valores IR en fuerzas vectoriales.

### Normalización de Sensores IR

Cada sensor IR tiene una sensibilidad diferente. Para poder usar umbrales uniformes, normalizamos las lecturas:

$$IR_{normalizado} = \frac{IR_{real}}{factor_{sensor}}$$

Los factores de sensibilidad se obtuvieron mediante calibración experimental con obstáculos a 5 cm:

| Sensor | Lectura a 5cm | Factor de Sensibilidad |
|--------|---------------|------------------------|
| 0 (lateral izq.) | 1382 | 1.382 |
| 1 (intermedio izq.) | 1121 | 1.121 |
| 2 (frontal izq.) | 270 | 0.270 |
| 3 (central) | 1045 | 1.045 |
| 4 (frontal der.) | 896 | 0.896 |
| 5 (intermedio der.) | 672 | 0.672 |
| 6 (lateral der.) | 901 | 0.901 |

Con esta normalización, todos los sensores producen aproximadamente IR = 1000 cuando detectan un obstáculo a 5 cm, permitiendo usar umbrales consistentes en todo el sistema.

### Conversión IR a Distancia

Convertimos las lecturas IR normalizadas a distancias estimadas usando un modelo físico mejorado con diferentes exponentes según el rango:

$$d_{obstaculo} = 5.0 \cdot \left(\frac{1000}{IR_{normalizado}}\right)^{p}$$

El exponente $p$ varía según el rango de la lectura IR normalizada:

| Rango de $IR_{norm}$ | Exponente $p$ | Distancia aprox. |
|---------------------|---------------|------------------|
| ≥ 1000 | N/A | 5.0 cm (saturado) |
| 60 - 1000 | 0.65 | 5-25 cm |
| < 60 | 0.70 | 25-60 cm |

**Implementación:**
```python
if ir_normalized >= 1000:
    distance = 5.0
elif ir_normalized >= 60:
    distance = 5.0 * math.pow(1000.0 / ir_normalized, 0.65)
else:
    distance = 5.0 * math.pow(1000.0 / ir_normalized, 0.70)
```

Además, aplicamos compensación por el ángulo del sensor, ya que los sensores laterales tienden a subestimar la distancia frontal efectiva:

$$d_{compensada} = d_{obstaculo} \cdot factor_{angulo}$$

| Ángulo del sensor | Factor de compensación |
|-------------------|------------------------|
| ±65° (laterales extremos) | 1.15 |
| ±38° (intermedios) | 1.08 |
| ±20° (frontales laterales) | 1.03 |
| < ±15° (frontal central) | 1.00 |

**Implementación:**
```python
if sensor_angle_deg > 50:
    distance *= 1.15
elif sensor_angle_deg > 30:
    distance *= 1.08
elif sensor_angle_deg > 15:
    distance *= 1.03
```

### Cálculo de Clearance

El *clearance* es el concepto fundamental del sistema repulsivo. Representa la distancia libre disponible después de considerar el radio físico del robot:

$$clearance = d_{obstaculo} - r_{robot}$$

**Parámetros:**
- $r_{robot} = 17.095$ cm (radio físico del robot)

**Implementación:**
```python
clearance = d_obstacle - config.ROBOT_RADIUS_CM
```

### Fuerza Repulsiva por Clearance

La fuerza repulsiva se calcula mediante una función por casos que depende del clearance disponible:

$$F_{repulsiva} = \begin{cases}
k_{rep} \cdot 10.0 & \text{si } clearance < 1.0 \text{ cm} \\
k_{rep} \cdot \left(\frac{1}{clearance} - \frac{1}{d_{safe}}\right)^2 & \text{si } 1.0 \leq clearance < d_{safe} \\
k_{rep} \cdot \left(\frac{d_{safe}}{clearance}\right)^3 \cdot factor_{alcance} & \text{si } clearance \geq d_{safe}
\end{cases}$$

**Parámetros:**
- $k_{rep} = 300.0$ (definido en `config.K_REPULSIVE`)
- $d_{safe} = 20.0$ cm (clearance mínimo seguro, definido en `config.D_SAFE`)
- $factor_{alcance} = 1.0 - \frac{d_{obstaculo}}{d_{influencia}}$
- $d_{influencia} = 100.0$ cm (definido en `config.D_INFLUENCE`)

**Implementación:**
```python
d_safe = config.D_SAFE  # 20cm

if clearance < 1.0:
    force_magnitude = k_rep * 10.0
elif clearance < d_safe:
    term = (1.0 / clearance) - (1.0 / d_safe)
    force_magnitude = k_rep * (term ** 2)
else:
    factor_alcance = 1.0 - (d_obstacle / d_influence)
    force_magnitude = k_rep * math.pow(d_safe / clearance, 3.0) * factor_alcance
```

La dirección de la fuerza repulsiva apunta en dirección opuesta al obstáculo detectado:

$$\vec{F}_{rep} = F_{mag} \cdot (\cos(\theta_{sensor} + \pi), \sin(\theta_{sensor} + \pi))$$

donde $\theta_{sensor}$ es la dirección global del sensor en el marco de referencia mundial.

## Combinación de Fuerzas

Las fuerzas atractivas y repulsivas se combinan vectorialmente para obtener la dirección resultante de navegación. El sistema calcula pesos dinámicos que balancean ambas fuerzas según la proximidad de obstáculos.

### Cálculo de Pesos Dinámicos

El peso de la fuerza repulsiva se calcula basándose en su magnitud:

$$w_{rep} = \min\left(\frac{|\vec{F}_{repulsiva}|}{3.5}, 0.85\right)$$

$$w_{att} = 1.0 - w_{rep}$$

**Parámetros:**
- Divisor de normalización = 3.5 (ajustado experimentalmente)
- Máximo peso repulsivo = 0.85 (85% del total)

**Implementación:**
```python
f_rep_mag = math.hypot(fx_rep, fy_rep)
weight_rep = min(f_rep_mag / 3.5, 0.85)
weight_att = 1.0 - weight_rep
```

Esto garantiza que cuando hay obstáculos cercanos, la fuerza repulsiva domina (hasta 85%), pero siempre mantiene al menos un 15% de influencia atractiva para evitar quedar atrapado en mínimos locales.

### Combinación Vectorial

Las fuerzas se combinan mediante promedio ponderado en componentes cartesianas:

$$\vec{F}_{total} = w_{att} \cdot \vec{F}_{atractiva} + w_{rep} \cdot \vec{F}_{repulsiva}$$

**En componentes:**

$$F_{total,x} = w_{att} \cdot \cos(\theta_{atractivo}) + w_{rep} \cdot \cos(\theta_{repulsivo})$$

$$F_{total,y} = w_{att} \cdot \sin(\theta_{atractivo}) + w_{rep} \cdot \sin(\theta_{repulsivo})$$

**Ángulo resultante:**

$$\theta_{deseado} = \arctan2(F_{total,y}, F_{total,x})$$

**Implementación:**
```python
combined_x = weight_att * math.cos(desired_angle_att) + weight_rep * math.cos(angle_rep)
combined_y = weight_att * math.sin(desired_angle_att) + weight_rep * math.sin(angle_rep)
desired_angle = math.atan2(combined_y, combined_x)
```

La velocidad lineal base se calcula usando la función de potencial atractivo seleccionada, y luego se ajusta según la influencia repulsiva y el clearance disponible.

## Control de Velocidad Dinámico

El sistema implementa un control dinámico de velocidad multicapa que ajusta la velocidad del robot según las condiciones del entorno, garantizando seguridad sin sacrificar eficiencia.

### Rampa de Aceleración

Para evitar movimientos bruscos que puedan causar deslizamiento de las ruedas o inestabilidad, limitamos el cambio máximo de velocidad por iteración:

$$\Delta v_{max} = a_{ramp} \cdot \Delta t$$

**Parámetros:**
- $a_{ramp} = 10.0$ cm/s² (definido en `config.ACCEL_RAMP_CM_S2`)
- $\Delta t = 0.05$ s (período de control, `config.CONTROL_DT`)

**Restricción aplicada:**

$$v_{nueva} = \begin{cases}
v_{anterior} + \Delta v_{max} & \text{si } v_{deseada} > v_{anterior} + \Delta v_{max} \\
v_{deseada} & \text{en otro caso}
\end{cases}$$

**Implementación:**
```python
max_delta_v = config.ACCEL_RAMP_CM_S2 * config.CONTROL_DT
if v_linear > _last_v_linear:
    v_linear = min(v_linear, _last_v_linear + max_delta_v)
_last_v_linear = v_linear
```

### Distancia de Frenado Predictiva

El sistema calcula continuamente la distancia necesaria para frenar completamente desde la velocidad actual:

$$d_{frenado} = \frac{v_{actual}^2}{2 \cdot a_{decel}}$$

**Parámetros:**
- $a_{decel} = 20.0$ cm/s² (tasa de desaceleración segura)

**Implementación:**
```python
decel_rate = 20.0
brake_distance = (current_v ** 2) / (2 * decel_rate)
```

### Clearance Efectivo

Combinamos el clearance real frontal con la distancia de frenado para obtener un clearance efectivo que considera la inercia del robot:

$$clearance_{efectivo} = clearance_{real} - d_{frenado}$$

**Implementación:**
```python
effective_clearance = min_clearance_front - brake_distance
```

Este concepto es crucial porque permite al robot anticipar situaciones de peligro antes de que se vuelvan críticas.

### Sistema de Niveles de Seguridad

La velocidad máxima permitida se determina según el clearance efectivo y el clearance real mínimo frontal mediante una tabla de decisión con cinco niveles:

| Condición | Nivel de Seguridad | $v_{max}$ permitida |
|-----------|-------------------|---------------------|
| $clearance_{efectivo} < 5$ cm o $clearance_{real} < 3$ cm | EMERGENCIA | 8 cm/s |
| $clearance_{efectivo} < 12$ cm o $clearance_{real} < 8$ cm | CRÍTICO | 15 cm/s |
| $clearance_{efectivo} < 20$ cm o $clearance_{real} < 15$ cm | ADVERTENCIA | 25 cm/s |
| $clearance_{efectivo} < 30$ cm o $clearance_{real} < 25$ cm | PRECAUCIÓN | 35 cm/s |
| $clearance_{efectivo} \geq 30$ cm y $clearance_{real} \geq 25$ cm | LIBRE | 38 cm/s |

**Implementación:**
```python
if effective_clearance < 5.0 or min_clearance_front < 3.0:
    v_max_allowed = 8.0  # EMERGENCY
elif effective_clearance < 12.0 or min_clearance_front < 8.0:
    v_max_allowed = 15.0  # CRITICAL
elif effective_clearance < 20.0 or min_clearance_front < 15.0:
    v_max_allowed = 25.0  # WARNING
elif effective_clearance < 30.0 or min_clearance_front < 25.0:
    v_max_allowed = 35.0  # CAUTION
else:
    v_max_allowed = 38.0  # CLEAR
```

**Boost por Gap Navegable**: Si se detecta un gap navegable, la velocidad permitida aumenta:
- Gap muy ancho (> 64 cm): +30% de velocidad (máximo 38 cm/s)
- Gap ancho (> 49 cm): +15% de velocidad (máximo 38 cm/s)

### Reducción por Error Angular

La velocidad también se ajusta según qué tan alineado esté el robot con la dirección deseada. El factor de reducción se basa en el coseno del error angular:

$$factor_{angular} = \max(\cos(\theta_{error}), factor_{min})$$

donde $factor_{min}$ depende de la distancia al objetivo:

$$factor_{min} = \begin{cases}
0.6 & \text{si } d > 50 \text{ cm} \\
0.4 & \text{si } 20 \leq d \leq 50 \text{ cm} \\
0.2 & \text{si } d < 20 \text{ cm}
\end{cases}$$

**Velocidad ajustada:**

$$v_{lineal} = v_{base} \cdot factor_{angular}$$

**Implementación:**
```python
angle_factor = math.cos(angle_error)

if distance > 50.0:
    min_factor = 0.6
elif distance > 20.0:
    min_factor = 0.4
else:
    min_factor = 0.2

if angle_factor < min_factor:
    angle_factor = min_factor

v_linear *= angle_factor
```

Esto permite al robot girar suavemente hacia la dirección correcta sin perder estabilidad.

## Cinemática Diferencial

El robot iRobot Create 3 utiliza tracción diferencial, donde cada rueda puede controlarse independientemente. Convertimos las velocidades lineal y angular deseadas a velocidades individuales de rueda.

### Conversión a Velocidades de Rueda

Las ecuaciones de cinemática diferencial relacionan la velocidad lineal $v$ y angular $\omega$ con las velocidades de las ruedas izquierda y derecha:

$$v_{izquierda} = v_{lineal} - \frac{L}{2} \cdot \omega$$

$$v_{derecha} = v_{lineal} + \frac{L}{2} \cdot \omega$$

**Parámetros:**
- $L = 23.5$ cm (wheelbase, distancia entre ruedas, definida en `config.WHEEL_BASE_CM`)
- $\omega$ = velocidad angular en rad/s

**Implementación:**
```python
half_base = config.WHEEL_BASE_CM / 2.0
v_left = v_linear - half_base * omega
v_right = v_linear + half_base * omega
```

### Cálculo de Velocidad Angular

La velocidad angular se calcula proporcionalmente al error angular hacia la dirección deseada:

$$\omega = k_{ang} \cdot \theta_{error}$$

**Parámetros:**
- $k_{ang} = 3.0$ (definido en `config.K_ANGULAR`)
- $\theta_{error}$ = error angular normalizado en radianes

**Normalización del error angular:**

El error angular se normaliza al rango $(-\pi, \pi]$ usando la función de envolvimiento:

$$\theta_{error} = \text{wrap}_\pi(\theta_{deseado} - \theta_{actual})$$

```python
def _wrap_pi(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad
```

**Saturación de velocidad angular:**

Para evitar giros excesivamente rápidos que puedan desestabilizar el robot:

$$\omega_{max} = \frac{W_{max}}{L/2}$$

$$\omega_{saturado} = \max(-\omega_{max}, \min(\omega_{max}, \omega))$$

donde $W_{max} = 10.0$ cm/s (definido en `config.W_MAX_CM_S`).

**Implementación:**
```python
omega = k_ang * angle_error
omega_max_rad_s = config.W_MAX_CM_S / (config.WHEEL_BASE_CM / 2.0)
omega = max(-omega_max_rad_s, min(omega_max_rad_s, omega))
```

### Restricción de Navegación en Arco

El sistema implementa una restricción crítica para garantizar que el robot SIEMPRE navega en arco (ambas ruedas hacia adelante) y NUNCA gira sobre su propio eje. Esta restricción se aplica limitando la velocidad angular máxima permitida según la velocidad lineal actual:

$$\omega_{max\_arco} = \frac{v_{lineal} - v_{min\_rueda}}{L/2}$$

donde $v_{min\_rueda}$ es la velocidad mínima garantizada de la rueda más lenta, que depende de la distancia al objetivo:

$$v_{min\_rueda} = \begin{cases}
4.0 \text{ cm/s} & \text{si } d > 30 \text{ cm} \\
2.0 \text{ cm/s} & \text{si } 10 \leq d \leq 30 \text{ cm} \\
0.0 \text{ cm/s} & \text{si } d < 10 \text{ cm}
\end{cases}$$

**Restricción aplicada:**

$$|\omega| \leq \omega_{max\_arco}$$

**Implementación:**
```python
if distance > 30.0:
    min_wheel_speed = 4.0
elif distance > 10.0:
    min_wheel_speed = 2.0
else:
    min_wheel_speed = 0.0

if distance > config.TOL_DIST_CM and v_linear > min_wheel_speed:
    max_omega_for_arc = (v_linear - min_wheel_speed) / half_base
    if abs(omega) > max_omega_for_arc:
        omega = math.copysign(max_omega_for_arc, omega)
```

Esta restricción garantiza movimiento fluido y natural del robot, evitando giros bruscos sobre el eje que podrían causar deslizamiento o confusión en la odometría.

## Detección de Gaps Navegables

El sistema detecta espacios entre obstáculos (gaps) por donde el robot puede pasar de forma segura. Esta capacidad es crucial para navegar en entornos con múltiples obstáculos sin detenerse innecesariamente.

### Geometría de Gaps

Un gap se forma cuando hay dos sensores adyacentes que detectan obstáculos, pero los sensores entre ellos reportan espacio libre. Para calcular el ancho del gap, primero estimamos las posiciones de los obstáculos en el marco de referencia local del robot.

**Posiciones de obstáculos en marco local:**

$$x_i = d_i \cdot \sin(\alpha_i)$$
$$y_i = d_i \cdot \cos(\alpha_i)$$

donde:
- $d_i$ = distancia estimada al obstáculo por sensor $i$
- $\alpha_i$ = ángulo del sensor $i$ desde el frente del robot

**Ancho del gap:**

La distancia euclidiana entre dos obstáculos detectados define el ancho del gap:

$$ancho_{gap} = \sqrt{(x_i - x_j)^2 + (y_i - y_j)^2}$$

**Implementación:**
```python
obs_i_local_x = dist_i * math.sin(angle_i_rad)
obs_i_local_y = dist_i * math.cos(angle_i_rad)

obs_j_local_x = dist_j * math.sin(angle_j_rad)
obs_j_local_y = dist_j * math.cos(angle_j_rad)

gap_width = math.hypot(
    obs_i_local_x - obs_j_local_x,
    obs_i_local_y - obs_j_local_y
)
```

### Criterio de Navegabilidad

Un gap es considerado navegable si su ancho es suficientemente mayor que el diámetro del robot para permitir paso seguro:

$$navegable = ancho_{gap} \geq 65 \text{ cm}$$

Este valor (definido en `config.GAP_MIN_WIDTH_CM`) considera el diámetro del robot (34.19 cm) más un margen de seguridad de aproximadamente 30 cm.

**Implementación:**
```python
is_navigable = gap_width >= config.GAP_MIN_WIDTH_CM  # 65.0 cm
```

### Reducción de Fuerza en Gaps

Cuando se detecta un gap navegable, las fuerzas repulsivas de los obstáculos que forman los bordes del gap se reducen para permitir que el robot pase por el espacio sin desviarse excesivamente:

$$F_{rep\_gap} = F_{rep} \cdot factor_{reduccion}$$

donde $factor_{reduccion} = 0.3$ (definido en `config.GAP_REPULSION_REDUCTION_FACTOR`).

**Implementación:**
```python
if gap.get('is_navigable', False):
    if i == left_idx or i == right_idx:
        force_magnitude *= config.GAP_REPULSION_REDUCTION_FACTOR
```

Esto significa que los obstáculos laterales del gap solo ejercen el 30% de su fuerza repulsiva normal, permitiendo al robot navegar confiadamente por el espacio disponible.

## Características Avanzadas

### Sistema de Escape de Trampas

El sistema incluye un modo especial para escapar de situaciones de trampa en C (mínimos locales) donde hay obstáculos adelante, izquierda y derecha simultáneamente.

**Condición de detección:**

$$trampa = (n_{sensores\_bloqueados} \geq 5) \land \neg gap_{navegable}$$

donde $n_{sensores\_bloqueados}$ es el número de sensores con $IR_{norm} \geq 100$.

**Implementación:**
```python
trapped_sensor_count = sum(1 for ir in normalized_ir if ir >= 100)
is_trapped = (trapped_sensor_count >= 5) and not navigable_gap_detected
```

**Modificadores cuando está atrapado:**

El sistema ajusta las ganancias para favorecer la exploración y el escape:

$$k_{att\_efectivo} = k_{att} \cdot 0.3$$
$$k_{rep\_efectivo} = k_{rep} \cdot 1.5$$
$$k_{ang\_efectivo} = k_{ang} \cdot 1.5$$
$$v_{min\_garantizada} = 4.0 \text{ cm/s}$$

**Implementación:**
```python
if is_trapped:
    k_lin_effective = k_lin * 0.3
    k_rep_effective = k_rep * 1.5
    k_ang_adjusted = k_ang * 1.5
    if v_base < 4.0:
        v_base = 4.0
```

Esto reduce temporalmente la fuerza atractiva (a 30%), aumenta la fuerza repulsiva (50% adicional), mantiene velocidad mínima hacia adelante para explorar alternativas, y aumenta la capacidad de giro para encontrar la apertura.

### Transformación de Coordenadas

El sistema implementa una transformación completa de coordenadas que permite trabajar en un sistema mundial especificado en `points.json`, independientemente de la orientación inicial del robot:

1. **Rotación**: Las coordenadas de odometría se rotan según la diferencia entre el heading real y el deseado
2. **Traslación**: Se suman los offsets de posición inicial
3. **Corrección de heading**: Se aplica un offset angular para convertir al sistema mundial

Esto permite que el robot funcione correctamente sin importar cómo esté orientado físicamente al inicio.

## Configuración y Uso

### Configuración de Puntos de Navegación

Antes de ejecutar cualquiera de los scripts principales, necesitamos configurar los puntos de navegación:

```bash
python utils/point_manager.py
```

Este script permite controlar el robot manualmente mediante teclado y marcar las posiciones inicial y final usando los botones físicos del robot. Genera el archivo `points.json` en la carpeta `data`.

También podemos editar manualmente el archivo JSON para modificar los valores sin necesidad de ejecutar el script nuevamente.

### Ejecución de la Parte 01

```bash
python PRM01_P01.py --potential linear
python PRM01_P01.py --potential quadratic
python PRM01_P01.py --potential conic
python PRM01_P01.py --potential exponential
```

### Ejecución de la Parte 02

```bash
python PRM01_P02.py --potential conic
python PRM01_P02.py --potential conic --k-rep 500 --d-influence 80
```

### Opciones Adicionales

Ambos scripts aceptan los siguientes argumentos:
- `--debug`: Muestra información detallada cada 10 iteraciones
- `--robot`: Especifica el nombre Bluetooth del robot (por defecto: "C3_UIEC_Grupo1")
- `--points`: Ruta al archivo JSON con puntos de navegación (por defecto: "data/points.json")

Para la Parte 02, también están disponibles:
- `--k-rep`: Ganancia repulsiva (por defecto: 300.0)
- `--d-influence`: Distancia de influencia repulsiva en cm (por defecto: 100.0)

## Módulos del Sistema

### config.py

Contiene todos los parámetros configurables del sistema centralizados en un solo lugar. Define velocidades máximas, ganancias de control, umbrales de sensores, y parámetros específicos para cada función de potencial. Facilita la calibración y ajuste del sistema sin modificar el código principal.

### potential_fields.py

Implementa las funciones de cálculo de potencial tanto atractivo como repulsivo:
- Cuatro variantes de potencial atractivo (lineal, cuadrática, cónica, exponencial)
- Conversión de lecturas IR a posiciones de obstáculos usando modelo físico mejorado
- Cálculo de fuerzas repulsivas basadas en clearance
- Detección de espacios navegables (gaps) entre obstáculos
- Combinación vectorial de fuerzas atractivas y repulsivas
- Sistema de escape de trampas en C (mínimos locales)

### safety.py

Proporciona funciones de seguridad que protegen al robot:
- Saturación de velocidades a rangos seguros del hardware
- Detección temprana de obstáculos mediante análisis de sensores IR
- Manejo de colisiones físicas mediante bumpers
- Reducción progresiva de velocidad según proximidad de obstáculos

### sensor_logger.py

Implementa un sistema de monitoreo asíncrono que imprime periódicamente el estado de todos los sensores del robot durante la navegación:
- Posición odométrica (con transformación al sistema mundial)
- Lecturas de los siete sensores IR
- Estado de bumpers izquierdo y derecho
- Nivel de batería
- Análisis de seguridad con niveles de peligro

### velocity_logger.py

Registra todos los datos relevantes de la navegación en archivos CSV con timestamps únicos. Los archivos contienen:
- Trayectoria completa (posición y orientación)
- Velocidades calculadas y aplicadas
- Fuerzas atractivas y repulsivas
- Información sobre obstáculos detectados
- Niveles de seguridad
- Errores de distancia y orientación

## Salida y Análisis

Durante la ejecución, el sistema genera archivos CSV en la carpeta `logs` con nombres que incluyen el tipo de potencial utilizado y un timestamp. Estos archivos contienen información detallada de cada iteración del bucle de control (20 Hz).

### Análisis Comparativo

Para analizar estos datos de forma comparativa:

```bash
python analysis/analyze_results.py
```

Este script procesa automáticamente todos los archivos CSV generados y calcula métricas clave:
- Tiempo total de navegación
- Error final de posición
- Distancia recorrida total
- Velocidades promedio y máximas
- Eficiencia de la trayectoria

La salida muestra una tabla comparativa que permite identificar qué función de potencial tuvo mejor desempeño según diferentes criterios.

### Visualización del Sistema de Seguridad

Para visualizar el funcionamiento del sistema de seguridad:

```bash
python analysis/visualize_safety.py
```

Este script genera tres gráficos:
1. Relación entre valores IR y velocidades máximas permitidas
2. Estimación de distancias basada en el modelo físico de los sensores
3. Tabla comparativa visual de los diferentes niveles de seguridad

Las imágenes generadas se guardan en la carpeta `images`.

## Parámetros Principales

Los parámetros principales del sistema están definidos en `config.py` y han sido calibrados experimentalmente para lograr un comportamiento seguro y efectivo:

### Parámetros Físicos del Robot
- **Radio del robot** ($r_{robot}$): $17.095$ cm
- **Diámetro del robot**: $34.19$ cm
- **Wheelbase** ($L$): $23.5$ cm (distancia entre ruedas)

### Parámetros de Control de Velocidad
- **Velocidad máxima** ($v_{max}$): $38$ cm/s (reducida para más tiempo de reacción)
- **Velocidad máxima de rueda** ($W_{max}$): $10.0$ cm/s
- **Rampa de aceleración** ($a_{ramp}$): $10.0$ cm/s²
- **Tasa de desaceleración** ($a_{decel}$): $20.0$ cm/s²
- **Período de control** ($\Delta t$): $0.05$ s (20 Hz)

### Ganancias de Campos de Potencial

**Atractivas:**
- **Lineal** ($k_{lin}$): $0.25$
- **Cuadrática** ($k_{quad}$): $0.05$
- **Cónica** ($k_{conic}$): $0.15$
- **Exponencial** ($k_{exp}$): $2.5$

**Repulsivas:**
- **Ganancia repulsiva** ($k_{rep}$): $300.0$ (aumentada para reacción temprana)
- **Ganancia angular** ($k_{ang}$): $3.0$ (aumentada para giros rápidos)

### Parámetros del Sistema Repulsivo
- **Distancia de influencia** ($d_{influencia}$): $100.0$ cm (detección temprana)
- **Clearance mínimo seguro** ($d_{safe}$): $20.0$ cm
- **Ancho mínimo de gap navegable**: $65.0$ cm
- **Factor de reducción de fuerza en gap**: $0.3$ (30% de fuerza normal)

### Tolerancias de Navegación
- **Tolerancia de distancia**: $5.0$ cm (para considerar que llegó al objetivo)
- **Tolerancia angular**: $0.17$ rad ($\approx 10°$)

### Factores de Sensibilidad IR

Los factores de sensibilidad normalizan las lecturas de cada sensor (calibrados a 5 cm):

| Sensor | Ángulo | Factor |
|--------|--------|--------|
| 0 (lateral izq.) | +65° | 1.382 |
| 1 (intermedio izq.) | +38° | 1.121 |
| 2 (frontal izq.) | +20° | 0.270 |
| 3 (central) | 0° | 1.045 |
| 4 (frontal der.) | -20° | 0.896 |
| 5 (intermedio der.) | -38° | 0.672 |
| 6 (lateral der.) | -65° | 0.901 |

Todos estos valores se encuentran centralizados en `src/config.py` para facilitar calibración y ajuste del sistema.

## Resultados y Conclusiones

El sistema implementado permite al robot navegar de forma autónoma desde una posición inicial hasta un objetivo, evitando obstáculos detectados mediante sensores IR. Las diferentes funciones de potencial ofrecen comportamientos distintos que pueden ser seleccionados según las características del entorno y los objetivos de navegación.

El sistema de seguridad basado en clearance efectivo y frenado predictivo garantiza tiempo suficiente de reacción ante obstáculos, mientras que la detección de gaps navegables permite al robot pasar por pasillos estrechos sin detenerse innecesariamente.

Los archivos CSV generados permiten análisis comparativo detallado para evaluar el rendimiento de cada función de potencial y ajustar parámetros según sea necesario para mejorar el comportamiento en diferentes condiciones de navegación.
