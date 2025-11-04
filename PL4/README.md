# Práctica 4 - Navegación con Campos de Potencial

**Autores:** Yago Ramos - Alan Salazar  
**Fecha:** 28 de octubre de 2025  
**Institución:** UIE - Robots Autónomos  
**Robot:** iRobot Create 3

## Introducción

Para esta práctica evaluada número 4 teníamos como desafío implementar un sistema completo de navegación autónoma para el robot Create 3 utilizando campos de potencial. El objetivo principal era lograr que el robot navegara desde una posición inicial hasta una posición final, primero usando únicamente un campo de potencial atractivo, y posteriormente combinando este campo atractivo con un campo repulsivo para evitar obstáculos detectados por los sensores infrarrojos.

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
│   ├── safety_ir_vs_vmax.png
│   ├── safety_ir_vs_distance.png
│   └── safety_table.png
└── logs/                 # Archivos CSV con datos de telemetría
```

## Parte 01 - Campo de Potencial Atractivo

En el primer código PRM01_P01.py buscamos implementar la navegación básica utilizando únicamente un campo de potencial atractivo. Este campo genera una fuerza que atrae al robot hacia la posición objetivo, calculando en cada iteración las velocidades de las ruedas necesarias para avanzar en esa dirección.

Este script funciona con los archivos del módulo src, específicamente con potential_fields.py que contiene las cuatro funciones de potencial que implementamos: lineal, cuadrática, cónica y exponencial. Cada una de estas funciones tiene características diferentes en cuanto a cómo la fuerza varía con la distancia al objetivo.

El script también integra el módulo safety.py para aplicar límites de seguridad a las velocidades calculadas, sensor_logger.py para monitorear el estado de los sensores durante la ejecución, y velocity_logger.py para registrar todos los datos de la navegación en un archivo CSV que nos permite analizar el comportamiento posteriormente.

Para ejecutar este script necesitamos primero haber configurado los puntos de navegación usando el script point_manager.py, que genera el archivo points.json en la carpeta data. Este archivo JSON contiene la estructura con dos puntos principales: q_i que representa la posición inicial del robot con coordenadas x e y en centímetros, y theta que es la orientación inicial del robot en grados (donde 0 grados apunta hacia el eje positivo X, y los ángulos crecen en sentido antihorario siguiendo la convención matemática estándar). El punto q_f contiene únicamente las coordenadas x e y del objetivo final, ya que no necesitamos especificar una orientación para la meta.

Adicionalmente, podemos manipular manualmente el archivo JSON para modificar estos valores sin necesidad de ejecutar el script point_manager.py nuevamente. Esto nos permite probar diferentes configuraciones de puntos de navegación editando directamente los valores de x, y y theta en el archivo. Una vez que tenemos este archivo configurado, podemos ejecutar PRM01_P01.py especificando qué tipo de función de potencial queremos usar mediante el argumento --potential.

## Parte 02 - Campo de Potencial Combinado

En el segundo código PRM01_P02.py extendimos la funcionalidad anterior para incluir un campo de potencial repulsivo que evita obstáculos. Esta implementación combina el campo atractivo hacia la meta con fuerzas repulsivas calculadas a partir de las lecturas de los sensores infrarrojos del robot.

La diferencia principal con respecto a PRM01_P01.py es que ahora utilizamos la función combined_potential_speeds() del módulo potential_fields.py en lugar de attractive_wheel_speeds(). Esta función toma en cuenta las lecturas de los sensores IR para calcular obstáculos en el entorno y generar fuerzas repulsivas que modifican la trayectoria del robot.

El sistema funciona leyendo continuamente los siete sensores infrarrojos del robot, estimando la posición de los obstáculos basándose en un modelo físico que relaciona la intensidad de la señal con la distancia, y luego calculando fuerzas repulsivas que se combinan vectorialmente con la fuerza atractiva hacia el objetivo. El resultado es una navegación que se ajusta dinámicamente para evitar colisiones mientras mantiene el objetivo de llegar a la meta.

Este script también permite ajustar parámetros del potencial repulsivo mediante argumentos de línea de comandos, como la ganancia repulsiva y la distancia de influencia, lo que nos permite experimentar con diferentes configuraciones según las características del entorno.

## Funciones de Potencial Implementadas

Implementamos cuatro funciones de potencial atractivo diferentes, cada una con características particulares que afectan el comportamiento del robot durante la navegación:

**Función Lineal:** F = k * d

Esta función genera una fuerza directamente proporcional a la distancia al objetivo. El comportamiento es predecible y directo, manteniendo una velocidad aproximadamente constante durante todo el trayecto una vez que se alcanza la velocidad máxima.

**Función Cuadrática:** F = k * d²

En esta función la fuerza crece con el cuadrado de la distancia, lo que significa que el robot acelera más agresivamente cuando está lejos del objetivo y desacelera de forma más suave cuando se acerca. Esto puede resultar en trayectos más rápidos pero requiere más control cerca de la meta.

**Función Cónica:** F = k * min(d, d_sat)

Esta función incluye una saturación a una distancia máxima determinada. Cuando el robot está más lejos que esta distancia de saturación, la velocidad se mantiene constante, y solo cuando se acerca comienza a reducir la velocidad. Esto es útil para navegación en espacios grandes donde queremos mantener velocidad constante en tramos largos.

**Función Exponencial:** F = k * (1 - e^(-d/λ))

La función exponencial presenta una convergencia asintótica, acelerando rápidamente al inicio pero desacelerando de forma muy suave conforme se acerca al objetivo. Esta característica puede ser útil cuando queremos un comportamiento más suave cerca de la meta.

## Configuración y Uso

Antes de ejecutar cualquiera de los scripts principales, necesitamos configurar los puntos de navegación. Para esto ejecutamos el script point_manager.py que nos permite controlar el robot manualmente mediante teclado y marcar las posiciones inicial y final usando los botones físicos del robot:

```bash
python utils/point_manager.py
```

Este script genera el archivo points.json en la carpeta data con las coordenadas de los puntos q_i (inicial) y q_f (final) que utilizaremos en la navegación.

Una vez configurados los puntos, podemos ejecutar el script de la Parte 01 con cualquiera de las funciones de potencial disponibles:

```bash
python PRM01_P01.py --potential linear
python PRM01_P01.py --potential quadratic
python PRM01_P01.py --potential conic
python PRM01_P01.py --potential exponential
```

Para la Parte 02, ejecutamos PRM01_P02.py con opciones similares, pero también podemos ajustar los parámetros del potencial repulsivo:

```bash
python PRM01_P02.py --potential conic
python PRM01_P02.py --potential conic --k-rep 500 --d-influence 30
```

Ambos scripts aceptan el argumento --debug para mostrar información detallada durante la ejecución, y --robot para especificar el nombre Bluetooth del robot si es diferente al configurado por defecto de nuestro grupo 01.

## Módulos del Sistema

El sistema está compuesto por varios módulos que trabajan juntos para proporcionar la funcionalidad completa:

**config.py:** Contiene todos los parámetros configurables del sistema centralizados en un solo lugar. Aquí definimos velocidades máximas, ganancias de control, umbrales de sensores, y parámetros específicos para cada función de potencial. Esto facilita la calibración y ajuste del sistema sin modificar el código principal.

**potential_fields.py:** Implementa las funciones de cálculo de potencial tanto atractivo como repulsivo. Contiene las cuatro variantes de potencial atractivo, la función para convertir sensores IR en posiciones de obstáculos, el cálculo de fuerzas repulsivas, y la combinación de ambas fuerzas para generar velocidades de rueda.

**safety.py:** Proporciona funciones de seguridad que protegen al robot limitando las velocidades a rangos seguros y detectando obstáculos mediante análisis de los sensores IR. Incluye un sistema de umbrales escalonados que reduce gradualmente la velocidad según la proximidad de obstáculos detectados.

**sensor_logger.py:** Implementa un sistema de monitoreo asíncrono que imprime periódicamente el estado de todos los sensores del robot durante la navegación, incluyendo posición odométrica, lecturas de sensores IR, estado de bumpers y nivel de batería.

**velocity_logger.py:** Registra todos los datos relevantes de la navegación en archivos CSV con timestamps únicos. Estos archivos contienen la trayectoria completa, velocidades calculadas, fuerzas aplicadas, y otra información que nos permite analizar el comportamiento del sistema posteriormente.

## Salida y Análisis

Durante la ejecución, el sistema genera archivos CSV en la carpeta logs con nombres que incluyen el tipo de potencial utilizado y un timestamp. Estos archivos contienen información detallada de cada iteración del bucle de control, incluyendo posición, velocidades, distancias, errores angulares, y en el caso de PRM01_P02.py, información sobre las fuerzas repulsivas y obstáculos detectados.

Para analizar estos datos de forma comparativa, desarrollamos el script analyze_results.py ubicado en la carpeta analysis. Este script procesa automáticamente todos los archivos CSV generados en la carpeta logs y calcula métricas clave como tiempo total de navegación, error final, distancia recorrida, y velocidades promedio y máximas. La salida muestra una tabla comparativa que nos permite identificar qué función de potencial tuvo mejor desempeño según diferentes criterios.

Para visualizar el funcionamiento del sistema de seguridad basado en umbrales escalonados, creamos el script visualize_safety.py también en la carpeta analysis. Este script genera tres gráficos que muestran la relación entre los valores de los sensores IR y las velocidades máximas permitidas, la estimación de distancias basada en el modelo físico de los sensores, y una tabla comparativa visual de los diferentes niveles de seguridad. Las imágenes generadas se guardan en la carpeta images.

Para ejecutar estos scripts de análisis:

```bash
python analysis/analyze_results.py
python analysis/visualize_safety.py
```

Esta información nos permite realizar análisis comparativos entre las diferentes funciones de potencial, evaluar el rendimiento del sistema, y ajustar parámetros según sea necesario para mejorar el comportamiento en diferentes condiciones de navegación.
