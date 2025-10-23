# RobotIcreate - Sistema de Navegación Autónoma para iRobot Create3

Este repositorio contiene el desarrollo completo de sistemas de navegación autónoma para el robot iRobot Create3, desarrollado como parte de la asignatura de Robots Autónomos en Ingeniería en Sistemas Inteligentes. El proyecto abarca desde ejercicios básicos de calibración hasta un sistema completo de mapeo y navegación reactiva con evasión de obstáculos.

## Estructura del Proyecto

### 📁 Calibracion/
**Propósito**: Documentación técnica de calibración de sensores IR
- `CALIBRACION_create3.md`: Datos experimentales detallados de calibración de los 7 sensores IR del Create3, incluyendo valores de referencia para diferentes ángulos de obstáculos (frontal, 45° izquierda/derecha, perpendiculares). Contiene análisis de rangos de detección, sensibilidad por sensor y recomendaciones para implementación de sistemas de evasión.

### 📁 examples/
**Propósito**: Ejemplos básicos del SDK de iRobot para aprendizaje y referencia
- `navigation.py`: Demostración de navegación por coordenadas usando `navigate_to()`, incluye patrón de cuadrado con indicadores LED y sonoros
- `manual_move.py`: Control manual con teclado (WASD) y logging completo de sensores (posición, acelerómetro, bumpers, IR, cliff, batería)
- `ir_proximity_*.py`: Varios ejemplos de uso de sensores IR para detección de obstáculos, luces y notas musicales
- `docking.py`, `get_position.py`, `nivel_bateria.py`: Utilidades básicas para docking, odometría y monitoreo de batería
- `touch_music.py`: Control por botones táctiles con reproducción musical
- `clif_sensors.py`: Manejo de sensores de caída (cliff sensors)

### 📁 PL1/ (Práctica de Laboratorio 1)
**Propósito**: Ejercicios de inspección y navegación básica
- `INSPECION.py`: Sistema completo de inspección de área con giro de 360°, detección de obstáculos y señalización LED/sonora
- `Parte_A.py`, `Parte_B.py`, `Parte_C.py`: Ejercicios específicos de navegación y detección
- `Ronda_II.py`: Segunda ronda de ejercicios de navegación

### 📁 PL2/ (Práctica de Laboratorio 2)
**Propósito**: Desarrollo de sistemas de detección y evasión de obstáculos
- `T02_Etapa01.py`: Detección precisa a 15cm con sensores IR frontales, incluye señalización y cálculo de distancia recorrida
- `T02_Etapa02.py`: Sistema de evasión de obstáculos con giros automáticos
- `T02_Etapa03.py`: Navegación hacia objetivos específicos con evasión
- `T02_Etapa04.py`: Sistema completo de navegación autónoma

### 📁 PL3/ (Práctica de Laboratorio 3)
**Propósito**: Exploración autónoma y mapeo del entorno
- `ejercicio1.py`: Sistema de exploración que registra lugares visitados usando sensores IR y odometría
- `ejercicio2.py`, `ejercicio3.py`: Ejercicios avanzados de mapeo y navegación
- `T03_Entorno*.jsonl`: Archivos de datos generados por el sistema de exploración

### 📁 Proyecto_Final/
**Propósito**: Sistema completo de navegación autónoma con mapeo y evasión inteligente

#### Componentes Principales:
- **`nav_menu.py`**: Interfaz gráfica principal (Tkinter) para navegación autónoma entre nodos, incluye selección de origen (Undock/Start Nodo), destino por ID o nombre, y control de seguridad
- **`teleop_mark_nodes.py`**: Sistema de teleoperación para mapeo manual del entorno, permite marcar nodos (G), crear aristas entre puntos y generar datos de navegación reales
- **`core/ir_avoid.py`**: Implementación del algoritmo Bug2 formalizado con sensores IR para navegación reactiva, incluye estados SEEK/WALL_FOLLOW, filtros IIR, histéresis y detección de atascos
- **`core/safety.py`**: Monitor de seguridad no intrusivo con frenado automático basado en sensores IR, bumpers y cliff
- **`core/telemetry.py`**: Sistema de logging a 10Hz que registra pose, comandos, batería y estado de sensores
- **`core/undock.py`**: Secuencia estándar para salir del dock con retroceso controlado y giro
- **`core/config_validator.py`**: Validador de configuración YAML con rangos y valores por defecto
- **`nodes_io.py`**: Gestión de persistencia de nodos y aristas en formato JSONL, incluye métricas agregadas y logging de intentos de navegación
- **`visualize_nodes.py`**: Herramienta de visualización del grafo de navegación con análisis de calidad de aristas

#### Datos y Configuración:
- **`config.yaml`**: Configuración completa del sistema (robot, motion, safety, telemetry, avoidance) con valores calibrados en laboratorio
- **`nodes/`**: Directorio de datos persistentes
  - `nodes.jsonl`: Nodos del mapa con coordenadas y metadatos
  - `edges.jsonl`: Aristas entre nodos con segmentos cinemáticos y métricas de calidad
  - `logs/`: Archivos CSV de telemetría, intentos de navegación y segmentos de aristas
- **`requirements.txt`**: Dependencias Python (irobot-edu-sdk, matplotlib, numpy, pyyaml, pynput)

### 📁 robot_voice_control/
**Propósito**: Sistema experimental de control por voz usando OpenAI Realtime API
- `main.py`: Implementación de control de robot mediante comandos de voz procesados por GPT, incluye mock de API del robot para pruebas
- `requirements.txt`: Dependencias específicas (openai, pyaudio, websocket, dotenv)

### 📁 nodes/
**Propósito**: Datos globales de nodos y logs del sistema
- Contiene archivos JSONL de nodos y logs de telemetría históricos del sistema

## Funcionalidades Principales

### Sistema de Navegación Autónoma
El proyecto implementa un sistema completo de navegación que combina:
- **Mapeo manual**: Teleoperación para crear mapas de nodos con datos reales de navegación
- **Navegación reactiva**: Algoritmo Bug2 con sensores IR para evasión inteligente de obstáculos
- **Seguridad integrada**: Monitor que previene colisiones usando múltiples sensores
- **Telemetría completa**: Logging detallado para análisis y optimización

### Calibración de Sensores
Documentación técnica completa de los 7 sensores IR del Create3, incluyendo:
- Valores de referencia para diferentes ángulos de obstáculos
- Análisis de sensibilidad y rangos de detección
- Recomendaciones para implementación de sistemas de evasión

### Progresión de Aprendizaje
El repositorio sigue una progresión pedagógica desde conceptos básicos hasta sistemas avanzados:
1. **Ejemplos básicos**: Familiarización con el SDK y sensores
2. **PL1**: Inspección y navegación básica
3. **PL2**: Detección y evasión de obstáculos
4. **PL3**: Exploración autónoma y mapeo
5. **Proyecto Final**: Sistema completo integrado

### Herramientas de Análisis
- Visualización de grafos de navegación con análisis de calidad
- Logging detallado de telemetría y intentos de navegación
- Métricas de rendimiento por arista y segmento

## Tecnologías Utilizadas

- **Python 3.x**: Lenguaje principal de desarrollo
- **irobot-edu-sdk**: SDK oficial de iRobot para Create3
- **Tkinter**: Interfaz gráfica para navegación
- **Matplotlib**: Visualización de datos y mapas
- **PyYAML**: Configuración del sistema
- **pynput**: Control de teclado para teleoperación
- **OpenAI API**: Control por voz (experimental)

## Características Técnicas Destacadas

### Algoritmo de Navegación
- Implementación formal del algoritmo Bug2 con sensores IR
- Estados SEEK (rumbo al objetivo) y WALL_FOLLOW (bordeo de obstáculos)
- Filtros IIR para estabilidad de sensores
- Histéresis para evitar oscilaciones
- Detección automática de atascos con recuperación

### Sistema de Seguridad
- Monitor no intrusivo con override manual
- Frenado automático basado en múltiples sensores
- Señalización visual (LED) y auditiva
- Integración completa con el sistema de navegación

### Persistencia de Datos
- Formato JSONL para nodos y aristas
- Logging CSV detallado de telemetría
- Métricas agregadas por arista
- Versionado de formatos de datos

Este proyecto representa un desarrollo completo y profesional de sistemas de navegación autónoma, desde la calibración básica hasta la implementación de algoritmos avanzados de evasión de obstáculos.
