# Práctica 4 - Navegación con Campos de Potencial

**Autores:** Alan Ariel Salazar, Yago Ramos Sánchez  
**Fecha de Finalización:** 6 de noviembre de 2025  
**Institución:** Universidad Intercontinental de la Empresa (UIE)  
**Asignatura:** Robots Autónomos  
**Profesor:** Eladio Dapena  
**Robot:** iRobot Create 3

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

## Funciones de Potencial Implementadas

Implementamos cuatro funciones de potencial atractivo diferentes, cada una con características particulares:

### Función Lineal

$$F_{atractiva} = k_{lin} \cdot d$$

Esta función genera una fuerza directamente proporcional a la distancia al objetivo. El comportamiento es predecible y directo, manteniendo una velocidad aproximadamente constante durante todo el trayecto una vez que se alcanza la velocidad máxima.

### Función Cuadrática

$$F_{atractiva} = k_{quad} \cdot \frac{d^2}{10}$$

En esta función la fuerza crece con el cuadrado de la distancia (con factor de normalización). El robot acelera más agresivamente cuando está lejos del objetivo y desacelera de forma más suave cuando se acerca.

### Función Cónica

$$F_{atractiva} = k_{conic} \cdot \min(d, d_{sat}) \cdot 2$$

donde $d_{sat} = 100$ cm es la distancia de saturación.

Esta función incluye una saturación a una distancia máxima determinada. Cuando el robot está más lejos que esta distancia de saturación, la velocidad se mantiene constante, y solo cuando se acerca comienza a reducir la velocidad. Útil para navegación en espacios grandes.

### Función Exponencial

$$F_{atractiva} = k_{exp} \cdot (1 - e^{-d/\lambda}) \cdot 20$$

donde $\lambda = 50$ cm es el parámetro de escala.

La función exponencial presenta una convergencia asintótica, acelerando rápidamente al inicio pero desacelerando de forma muy suave conforme se acerca al objetivo. Útil cuando queremos un comportamiento más suave cerca de la meta.

## Campo de Potencial Repulsivo

El campo repulsivo se calcula a partir de las lecturas de los sensores IR. Primero convertimos las lecturas IR a distancias estimadas usando un modelo físico mejorado:

$$d_{obstaculo} = 5.0 \cdot \left(\frac{1000}{IR_{normalizado}}\right)^{0.65}$$

donde $IR_{normalizado}$ es la lectura del sensor normalizada según su sensibilidad específica.

Luego calculamos el *clearance* (distancia libre después del radio del robot):

$$clearance = d_{obstaculo} - r_{robot}$$

donde $r_{robot} = 17.095$ cm es el radio del robot.

La fuerza repulsiva se calcula basándose en el clearance:

$$F_{repulsiva} = \begin{cases}
k_{rep} \cdot 10.0 & \text{si } clearance < 1.0 \text{ cm} \\
k_{rep} \cdot \left(\frac{1}{clearance} - \frac{1}{d_{safe}}\right)^2 & \text{si } 1.0 \leq clearance < d_{safe} \\
k_{rep} \cdot \left(\frac{d_{safe}}{clearance}\right)^3 \cdot factor_{alcance} & \text{si } clearance \geq d_{safe}
\end{cases}$$

donde $d_{safe} = 20$ cm es la distancia de seguridad mínima y $factor_{alcance} = 1.0 - \frac{d_{obstaculo}}{d_{influencia}}$.

## Combinación de Campos de Potencial

Las fuerzas atractivas y repulsivas se combinan vectorialmente para obtener la dirección resultante:

$$\vec{F}_{total} = w_{att} \cdot \vec{F}_{atractiva} + w_{rep} \cdot \vec{F}_{repulsiva}$$

donde los pesos se calculan como:

$$w_{rep} = \min\left(\frac{|\vec{F}_{repulsiva}|}{3.5}, 0.85\right)$$

$$w_{att} = 1.0 - w_{rep}$$

La velocidad lineal base se calcula usando la función de potencial atractivo seleccionada, y luego se ajusta según la influencia repulsiva y el clearance disponible.

## Control de Velocidad Dinámico

El sistema implementa un control dinámico de velocidad basado en el clearance frontal disponible. Primero estimamos la distancia de frenado necesaria:

$$d_{frenado} = \frac{v_{actual}^2}{2 \cdot a_{decel}}$$

donde $a_{decel} = 20$ cm/s² es la tasa de desaceleración segura.

Luego calculamos el clearance efectivo:

$$clearance_{efectivo} = clearance_{real} - d_{frenado}$$

La velocidad máxima permitida se determina según el clearance efectivo:

- **EMERGENCIA**: $clearance_{efectivo} < 5$ cm → $v_{max} = 8$ cm/s
- **CRÍTICO**: $clearance_{efectivo} < 12$ cm → $v_{max} = 15$ cm/s
- **ADVERTENCIA**: $clearance_{efectivo} < 20$ cm → $v_{max} = 25$ cm/s
- **PRECAUCIÓN**: $clearance_{efectivo} < 30$ cm → $v_{max} = 35$ cm/s
- **LIBRE**: $clearance_{efectivo} \geq 30$ cm → $v_{max} = 38$ cm/s

## Conversión a Velocidades de Rueda

Las velocidades lineal y angular se convierten a velocidades individuales de rueda usando la cinemática diferencial:

$$v_{izquierda} = v_{lineal} - \frac{L}{2} \cdot \omega$$

$$v_{derecha} = v_{lineal} + \frac{L}{2} \cdot \omega$$

donde $L = 23.5$ cm es la distancia entre las ruedas (wheelbase) y $\omega$ es la velocidad angular en rad/s.

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

## Características Avanzadas

### Detección de Gaps Navegables

El sistema detecta espacios entre obstáculos por donde el robot puede pasar. Un gap es considerado navegable si:
- Hay dos sensores adyacentes que detectan obstáculos
- Los sensores entre ellos reportan espacio libre
- El ancho del espacio es mayor que el diámetro del robot más margen de seguridad ($> 65$ cm)

Cuando se detecta un gap navegable, las fuerzas repulsivas de los obstáculos laterales se reducen para permitir el paso.

### Sistema de Escape de Trampas

El sistema incluye un modo especial para escapar de situaciones de trampa en C (mínimos locales) donde hay obstáculos adelante, izquierda y derecha simultáneamente:
- Detecta cuando 5 o más sensores detectan obstáculos simultáneamente
- Reduce temporalmente la fuerza atractiva (a 30%)
- Aumenta la fuerza repulsiva (50% adicional)
- Mantiene velocidad mínima hacia adelante para explorar alternativas
- Aumenta la capacidad de giro para encontrar la apertura

### Transformación de Coordenadas

El sistema implementa una transformación completa de coordenadas que permite trabajar en un sistema mundial especificado en `points.json`, independientemente de la orientación inicial del robot:

1. **Rotación**: Las coordenadas de odometría se rotan según la diferencia entre el heading real y el deseado
2. **Traslación**: Se suman los offsets de posición inicial
3. **Corrección de heading**: Se aplica un offset angular para convertir al sistema mundial

Esto permite que el robot funcione correctamente sin importar cómo esté orientado físicamente al inicio.

## Parámetros Principales

Los parámetros principales del sistema están definidos en `config.py`:

- **Velocidad máxima**: $38$ cm/s (reducida para dar más tiempo de reacción)
- **Ganancia repulsiva**: $300$ (aumentada para reacción más temprana)
- **Distancia de influencia**: $100$ cm (aumentada para detección temprana)
- **Distancia de seguridad**: $20$ cm (aumentada para mayor margen)
- **Ganancias atractivas**: Ajustadas específicamente para cada función de potencial
- **Ganancia angular**: $3.0$ (aumentada para reacciones de giro más rápidas)

Estos valores han sido calibrados experimentalmente para lograr un comportamiento seguro y efectivo.

## Resultados y Conclusiones

El sistema implementado permite al robot navegar de forma autónoma desde una posición inicial hasta un objetivo, evitando obstáculos detectados mediante sensores IR. Las diferentes funciones de potencial ofrecen comportamientos distintos que pueden ser seleccionados según las características del entorno y los objetivos de navegación.

El sistema de seguridad basado en clearance efectivo y frenado predictivo garantiza tiempo suficiente de reacción ante obstáculos, mientras que la detección de gaps navegables permite al robot pasar por pasillos estrechos sin detenerse innecesariamente.

Los archivos CSV generados permiten análisis comparativo detallado para evaluar el rendimiento de cada función de potencial y ajustar parámetros según sea necesario para mejorar el comportamiento en diferentes condiciones de navegación.
