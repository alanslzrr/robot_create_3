# Integración MCP - Robot Create3 Navegación Topológica

## Resumen

Este proyecto implementa un sistema completo de control por voz del robot iRobot Create3 mediante Model Context Protocol (MCP) e integración con OpenAI Realtime API. El sistema permite controlar la navegación topológica del robot mediante comandos de voz naturales, manteniendo toda la lógica de navegación robusta existente intacta.

## Arquitectura del Sistema

El sistema utiliza **FastMCP con transporte SSE (Server-Sent Events)** para una conexión robusta y persistente entre componentes. Consta de 5 componentes principales:

```
┌─────────────────┐    WebSocket    ┌─────────────────┐    WebSocket    ┌─────────────────┐
│                 │ ◄─────────────► │                 │ ◄─────────────► │                 │
│   Navegador     │    Audio/JSON   │  robot_bridge   │    Audio/JSON   │  OpenAI Realtime│
│   (index.html)  │                 │   (puerto 8000) │                 │       API       │
│                 │                 │                 │                 │                 │
└─────────────────┘                 └────────┬────────┘                 └─────────────────┘
                                             │
                                             │ SSE (HTTP)
                                             │
                                    ┌────────▼────────┐
                                    │                 │
                                    │   mcp_server    │
                                    │  (puerto 8001)  │
                                    │                 │
                                    └────────┬────────┘
                                             │
                                             │ Python directo
                                             │
                                    ┌────────▼────────┐     Bluetooth     ┌─────────────────┐
                                    │                 │ ◄───────────────► │                 │
                                    │ robot_service   │                   │  Robot Create3  │
                                    │                 │                   │                 │
                                    └─────────────────┘                   └─────────────────┘
```

### 1. robot_bridge.py - Servidor Bridge Principal (Puerto 8000)

Servidor FastAPI que actúa como puente bidireccional entre el navegador del usuario, OpenAI Realtime API y el servidor MCP del robot. Responsabilidades:

- Servidor HTTP en puerto 8000 con FastAPI
- Endpoint WebSocket `/ws` para comunicación con el navegador
- Conexión WebSocket a OpenAI Realtime API
- **Conexión SSE al servidor MCP** (más robusto que stdio)
- Traducción de protocolos: audio del browser → OpenAI, llamadas de herramientas OpenAI → MCP
- Manejo de transcripciones bidireccionales (usuario → IA, IA → usuario)
- Ejecución de herramientas MCP cuando OpenAI las solicita
- Envío de respuestas de voz de la IA al navegador
- Polling de telemetría cada 1 segundo (reducido de 500ms para evitar conflictos durante navegación)

Clase principal: `BridgeSession` que gestiona el ciclo de vida completo de una sesión de voz.

### 2. static/index.html (600 líneas) - Interfaz Web

Interfaz web completa para control por voz del robot. Funcionalidades:

- Captura de audio del micrófono del navegador mediante MediaRecorder API
- WebSocket client para comunicación con robot_bridge.py
- Visualización de transcripciones en tiempo real (usuario y asistente)
- Panel de estado del robot (nodo actual, nodo destino, estado de navegación)
- **NUEVO:** Telemetría en vivo (posición X, Y, orientación θ con actualización a 2 Hz)
- **NUEVO:** Detección automática de eventos de finalización de misión
- Logs del sistema en tiempo real
- Reproducción de respuestas de voz de la IA
- UI responsiva con CSS moderno y animaciones de actualización

### 3. robot_service.py (400+ líneas) - Controlador del Robot

Controlador central que encapsula toda la lógica de navegación. Responsabilidades:

- Gestión de conexión Bluetooth persistente al robot Create3
- Mantenimiento de estado interno (nodo actual, misión activa, estado, heading)
- **NUEVO:** Persistencia de heading en `last_state.json` para navegación consecutiva correcta
- **NUEVO:** Ejecución de navegación en `NavigationThread` con event loop dedicado (evita conflictos con FastMCP)
- **NUEVO:** Sistema de obtención de heading real del robot antes de iniciar navegación
- **NUEVO:** Deshabilitación de loggers (`disable_logging=True`) para evitar conflictos de event loop
- Integración con `CombinedPotentialNavigator` con parámetro `disable_logging` para modo MCP
- Planificación de rutas con algoritmo de Dijkstra sobre grafo topológico
- Construcción de waypoints intermedios desde el grafo
- Manejo de paradas de emergencia y cancelaciones

Clase principal: `RobotController` con métodos `start_navigation()`, `emergency_stop()`, `get_status()`, `set_telemetry_callback()`.

### 4. mcp_server.py - Servidor MCP (Puerto 8001)

Servidor MCP que expone las capacidades del robot como herramientas estándar MCP. Funcionalidades:

- **Servidor MCP con transporte SSE (Server-Sent Events)**
- Endpoint SSE en `http://localhost:8001/sse`
- Endpoint de mensajes en `http://localhost:8001/messages`
- Protocolo JSON-RPC 2.0 para comunicación
- 4 herramientas expuestas:
  - `list_available_locations()`: Lista nodos del mapa topológico
  - `navigate_robot(destination_id, origin_id)`: Inicia navegación a un nodo (**origin_id ahora REQUERIDO**)
  - `emergency_stop()`: Detiene el robot inmediatamente
  - `get_robot_status()`: Consulta estado actual del robot (incluye `last_heading`)
- Instancia persistente de `RobotController` durante toda la vida del servidor
- Logging estructurado con timestamps

### 5. Componentes Existentes (Sin Modificaciones)

- `PRM02_P02_EQUIPO1_grafos.py`: Clase `CombinedPotentialNavigator` (líneas 330-831) con navegación reactiva usando campos de potencial combinados
- `grafos/prueba.py`: Carga de grafo desde JSON, algoritmo de Dijkstra, estructura de datos del grafo
- `src/config.py`: Parámetros calibrados experimentalmente (K_REPULSIVE=300.0, D_INFLUENCE=100.0, etc.)
- `src/potential_fields.py`: Cálculo de fuerzas atractivas y repulsivas

## Flujo de Datos Completo

```
Usuario (Navegador Web)
    ↓ WebSocket (audio chunks PCM16)
robot_bridge.py (Puerto 8000)
    ↓ WebSocket (audio base64)
OpenAI Realtime API
    ↓ JSON (transcripciones, llamadas a herramientas)
robot_bridge.py
    ↓ SSE/HTTP (JSON-RPC 2.0)
mcp_server.py (Puerto 8001)
    ↓ Llamadas directas Python
robot_service.py
    ↓ Bluetooth + iRobot SDK
Robot Create3 Físico
```

### Flujo de Telemetría en Vivo (NUEVO)

```
Robot Create3 Físico
    ↓ Bluetooth (get_position(), get_ir_proximity())
robot_service.py
    ↓ Bucle de telemetría (5 Hz durante navegación)
    ↓ Callback con datos de posición/sensores
[Actualmente sin callback - se usa polling]
    ↑ Polling cada 500ms (2 Hz)
robot_bridge.py
    ↓ stdio JSON-RPC (get_robot_status)
mcp_server.py
    ↓ WebSocket JSON
Navegador Web
    ↓ Actualización UI en tiempo real
Usuario ve posición X, Y, θ en vivo
```

### Sistema de Telemetría en Vivo (NUEVO)

El sistema ahora incluye telemetría en tiempo real que envía la posición del robot al navegador durante la navegación:

### Arquitectura de Telemetría

**1. En robot_service.py:**
- Bucle de telemetría paralelo durante `_run_mission()`
- Se ejecuta a 5 Hz (cada 200ms) mientras `state == "NAVIGATING"`
- Lee posición del robot: `await robot.get_position()` → (x, y, heading)
- Lee sensores IR: `await robot.get_ir_proximity()` → array de 7 lecturas
- Invoca callback con datos estructurados

**2. En robot_bridge.py:**
- Tarea 3 en `communication_loop()`: `telemetry_polling()`
- Consulta `get_robot_status()` vía MCP cada 500ms (2 Hz)
- Envía eventos `telemetry` al navegador con posición actualizada
- Detecta automáticamente eventos de finalización de misión
- Envía eventos `mission_event` cuando una misión completa o falla

**3. En static/index.html:**
- Panel "Posición en Vivo" con valores X, Y, θ
- Actualización en tiempo real con animación visual
- Timestamp de última actualización
- Indicador de estado de navegación

### Datos de Telemetría

Cada paquete de telemetría incluye:

```json
{
  "timestamp": 123456.789,
  "position": {
    "x": 45.67,
    "y": 123.45,
    "theta": 90.0
  },
  "state": "NAVIGATING",
  "current_node": 0,
  "target_node": 3,
  "ir_sensors": [200, 150, 100, 50, 100, 150, 200],
  "mission_completed": false
}
```

### Eventos de Misión

Cuando una navegación termina, se envía automáticamente:

```json
{
  "event": "mission_completed",
  "success": true,
  "destination_node": 3,
  "position": {"x": 50.0, "y": 120.0, "theta": 0.0},
  "message": "Llegada exitosa al Nodo 3."
}
```

### Frecuencias de Actualización

- **Telemetría del robot → robot_service:** 5 Hz (cada 200ms) durante navegación
- **Polling bridge → MCP:** 1 Hz (cada 1 segundo) continuamente (reducido de 500ms para evitar conflictos)
- **Actualización UI navegador:** En tiempo real al recibir datos

Este diseño balancea responsividad vs overhead de red/CPU y evita conflictos de event loop durante navegación.

## Ejemplo de Flujo: Usuario dice "Vete al nodo 3"

1. Browser captura audio del micrófono y envía chunks vía WebSocket a robot_bridge.py
2. robot_bridge.py convierte audio a base64 y reenvía a OpenAI Realtime API
3. OpenAI transcribe: "Vete al nodo 3" y envía evento `conversation.item.input_audio_transcription.completed`
4. robot_bridge.py muestra transcripción en la interfaz web
5. OpenAI decide llamar herramienta y envía `response.function_call_arguments.done` con `name: "navigate_robot"` y `arguments: {"destination_id": 3, "origin_id": 0}`
6. robot_bridge.py ejecuta `mcp_session.call_tool("navigate_robot", args)` que comunica con mcp_server.py vía stdio
7. mcp_server.py llama `controller.start_navigation(destination_id=3, origin_id=0)`
8. robot_service.py valida parámetros, calcula ruta con Dijkstra, crea `CombinedPotentialNavigator` y lanza navegación en background
9. robot_service.py retorna mensaje inmediato: "RECIBIDO: Iniciando navegación desde Nodo 0 hacia Nodo 3"
10. El resultado se propaga de vuelta: robot_service.py → mcp_server.py → robot_bridge.py → OpenAI
11. OpenAI genera respuesta de voz: "De acuerdo, yendo al nodo 3"
12. robot_bridge.py recibe chunks de audio de OpenAI y los reenvía al navegador
13. Usuario escucha la respuesta mientras el robot se mueve físicamente
14. **NUEVO:** Mientras el robot navega:
    - robot_service.py envía telemetría cada 200ms (posición X, Y, θ)
    - robot_bridge.py consulta estado cada 500ms vía `get_robot_status()`
    - Navegador actualiza panel "Posición en Vivo" en tiempo real
    - Usuario ve el robot moverse en las coordenadas X, Y
15. **NUEVO:** Al llegar al destino:
    - robot_service.py envía evento `mission_completed`
    - Bridge detecta cambio de estado `NAVIGATING` → `IDLE`
    - Navegador muestra mensaje "✅ Misión completada" automáticamente
    - OpenAI puede ser notificado del evento para comentar (opcional)

## Instalación

### Dependencias Requeridas

```bash
pip install mcp fastapi uvicorn websockets python-dotenv pydub aiohttp
```

**CRÍTICO:** También necesitas `ffmpeg` instalado en tu sistema para la conversión de audio:
- **Windows:** Descargar de https://ffmpeg.org/ y agregar al PATH
- **Linux:** `sudo apt-get install ffmpeg`
- **macOS:** `brew install ffmpeg`

Dependencias ya existentes en el proyecto:
- `irobot-edu-sdk`: SDK oficial del Create3
- `asyncio`: Estándar en Python 3.7+

### Estructura de Archivos del Proyecto

```
FinalPROYECT/
├── start_services.bat           # Script de inicio para Windows CMD
├── start_services.ps1           # Script de inicio para PowerShell
├── mcp_server.py                # Servidor MCP con transporte SSE (puerto 8001)
├── robot_bridge.py              # Servidor bridge FastAPI (puerto 8000)
├── robot_service.py             # Controlador del robot
├── static/
│   └── index.html               # Interfaz web
├── last_state.json              # Estado persistente
├── .env                         # Configuración (ver abajo)
├── PRM02_P02_EQUIPO1_grafos.py  # Existente - Navegación (sin cambios)
├── grafos/
│   ├── prueba.py                # Existente - Manejo de grafo (sin cambios)
│   └── grafo.json               # Existente - Definición del mapa
└── src/
    ├── config.py                 # Existente - Parámetros calibrados (sin cambios)
    └── potential_fields.py       # Existente - Cálculo de fuerzas (sin cambios)
```

## Configuración Inicial

### 1. Configurar API Key de OpenAI

Crear archivo `.env` en la raíz del proyecto:

```
# OpenAI API Key (requerida)
OPENAI_API_KEY=sk-proj-tu-clave-aqui

# URL del servidor MCP (opcional, por defecto http://localhost:8001/sse)
MCP_SERVER_URL=http://localhost:8001/sse
```

Obtener clave en: https://platform.openai.com/api-keys

### 2. Verificar Estado del Robot (Opcional)

Si es la primera vez, el sistema no conocerá la ubicación inicial. Opciones:

- Proporcionar `origin_id` explícitamente en la primera navegación
- Editar `last_state.json` manualmente:
```json
{
  "current_node": 0,
  "last_message": "Posición inicial configurada manualmente"
}
```

## Uso del Sistema

### Opción 1: Inicio Automático (Recomendado)

Usar el script de inicio que abre ambos servidores automáticamente:

**Windows (PowerShell):**
```powershell
.\start_services.ps1
```

**Windows (CMD):**
```cmd
start_services.bat
```

### Opción 2: Inicio Manual

Abrir **dos terminales** y ejecutar en orden:

**Terminal 1 - Servidor MCP (puerto 8001):**
```bash
python mcp_server.py
```

Output esperado:
```
============================================================
SERVIDOR MCP - ROBOT CREATE3 NAVIGATOR
============================================================
Host: 0.0.0.0
Puerto: 8001
SSE Endpoint: http://0.0.0.0:8001/sse
Messages Endpoint: http://0.0.0.0:8001/messages
============================================================
[...] Controlador inicializado correctamente
Iniciando servidor MCP con transporte SSE...
```

**Terminal 2 - Robot Bridge (puerto 8000):**
```bash
python robot_bridge.py
```

Output esperado:
```
============================================================
ROBOT CREATE3 BRIDGE SERVER
============================================================
Servidor: http://0.0.0.0:8000
Interfaz Web: http://0.0.0.0:8000/
WebSocket: ws://0.0.0.0:8000/ws
Health Check: http://0.0.0.0:8000/health
============================================================
MCP Server URL: http://localhost:8001/sse
============================================================
OpenAI API Key configurada

Iniciando servidor...
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### Acceder a la Interfaz Web

1. Abrir navegador en: http://localhost:8000
2. Clic en "Iniciar Sesión de Voz"
3. Permitir acceso al micrófono cuando el navegador lo solicite
4. Comenzar a hablar con el robot

### Herramientas MCP Disponibles

### 1. list_available_locations()

Lista todos los nodos del mapa topológico con sus IDs numéricos y nombres descriptivos.

Retorna: String formateado con lista de ubicaciones
```
Lugares del Mapa:
- ID 0: inicio
- ID 1: izq_0
- ID 2: izq_1
...
```

Uso desde voz: "¿A dónde puedes ir?" o "Lista los nodos disponibles"

### 2. navigate_robot(destination_id: int, origin_id: int)

Inicia navegación del robot hacia un nodo destino. La función retorna inmediatamente y la navegación física continúa en background.

**IMPORTANTE:** `origin_id` es ahora **REQUERIDO**. Si no conoces el origen, llama primero a `get_robot_status()` para obtener `current_node`.

Parámetros:
- `destination_id`: ID numérico del nodo destino (obligatorio)
- `origin_id`: ID numérico del nodo origen (**obligatorio**). Usa `current_node` de `get_robot_status()` si no lo conoces.

Retorna: Mensaje de confirmación
```
"RECIBIDO: Iniciando navegación desde Nodo 0 ('inicio') hacia Nodo 3 ('izq_2')."
```

Comportamiento interno:
1. Valida que no haya misión activa (estado != "NAVIGATING")
2. Valida que ambos nodos estén en rango [0, graph.V-1]
3. **NUEVO:** Obtiene heading real del robot antes de iniciar (crítico para navegación consecutiva)
4. Calcula ruta óptima con Dijkstra
5. Construye q_i, waypoints, q_f desde coordenadas del grafo
6. **NUEVO:** Crea `CombinedPotentialNavigator` con `disable_logging=True` (evita conflictos de event loop)
7. **NUEVO:** Lanza navegación en `NavigationThread` con event loop dedicado (evita conflictos con FastMCP/uvicorn)
8. Actualiza estado: `state = "NAVIGATING"`, `target_node = destination_id`
9. **NUEVO:** Al finalizar, guarda `last_heading` en `last_state.json` para próxima navegación
10. Retorna mensaje inmediatamente (no bloquea)

Uso desde voz: "Vete al nodo 3" o "Navega al nodo 5 desde el nodo 0"

### 3. emergency_stop()

Detiene inmediatamente el robot y cancela cualquier misión de navegación activa.

Retorna: Mensaje confirmando acciones tomadas
```
"Parada de emergencia solicitada. Misión cancelada. Motores detenidos."
```

Comportamiento interno:
1. Cancela `mission_task` si existe (asyncio.Task)
2. Envía `robot.set_wheel_speeds(0, 0)` al hardware
3. Actualiza estado: `state = "IDLE"`, `target_node = None`
4. Guarda estado en `last_state.json`

Uso desde voz: "¡Para!" o "Detente" o "Alto"

### 4. get_robot_status()

Consulta el estado actual del robot para monitoreo.

Retorna: Diccionario con estado estructurado
```json
{
  "state": "NAVIGATING",
  "current_node": 0,
  "target_node": 3,
  "last_heading": 90.5,
  "robot_connected": true,
  "last_message": "Iniciando ruta: 0 -> 3.",
  "position": {"x": 0.0, "y": 0.0, "theta": 90.0},
  "mission_completed": false
}
```

Valores posibles de `state`:
- "IDLE": Robot en reposo, sin misión activa
- "NAVIGATING": Robot ejecutando navegación hacia target_node
- "ERROR": Error en última operación

Uso desde voz: "¿Cómo vas?" o "¿Cuál es tu estado?" o "¿Dónde estás?"

## Ejemplos de Uso

### Ejemplo 1: Primera Navegación (sin estado previo)

Usuario: "¿A dónde puedes ir?"
- OpenAI llama: `list_available_locations()`
- Respuesta: Lista de 15 nodos con nombres
- Usuario escucha: "Puedo ir a los siguientes lugares: inicio, izq_0, izq_1..."

Usuario: "Vete al nodo 3"
- OpenAI llama primero: `get_robot_status()` para obtener `current_node`
- Respuesta: `{"current_node": 0, "last_heading": 90.0, ...}`
- OpenAI llama: `navigate_robot(destination_id=3, origin_id=0)`
- **NUEVO:** Sistema obtiene heading real del robot (ej: 89.9°) antes de iniciar
- Robot calcula ruta: 0 → 5 → 1 → 2 → 3 (Dijkstra)
- Robot inicia navegación física en background usando heading real
- Usuario escucha: "De acuerdo, yendo al nodo 3"
- Al llegar, sistema guarda `current_node=3` y `last_heading=45.2°` en `last_state.json`

### Ejemplo 2: Navegaciones Subsecuentes

Usuario: "Ahora ve al nodo 5"
- OpenAI llama primero: `get_robot_status()` para obtener `current_node`
- Respuesta: `{"current_node": 3, "last_heading": 45.2, ...}`
- OpenAI llama: `navigate_robot(destination_id=5, origin_id=3)`
- **NUEVO:** Sistema obtiene heading real del robot (45.2°) antes de iniciar
- **NUEVO:** Usa heading real en lugar del theta del grafo (crítico para navegación consecutiva)
- Robot calcula ruta: 3 → 2 → 1 → 5
- Robot navega automáticamente con transformación de coordenadas correcta

### Ejemplo 3: Parada de Emergencia

Usuario: "¡Para!"
- OpenAI llama: `emergency_stop()` inmediatamente
- Robot cancela misión activa
- Robot envía `set_wheel_speeds(0, 0)` al hardware
- Estado cambia a "IDLE"
- Usuario escucha: "Parada de emergencia ejecutada"

### Ejemplo 4: Consulta de Estado

Usuario: "¿Cómo vas?"
- OpenAI llama: `get_robot_status()`
- Respuesta: `{"state": "NAVIGATING", "current_node": 3, "target_node": 5, ...}`
- Usuario escucha: "Estoy navegando hacia el nodo 5, actualmente desde el nodo 3"

## Testing

### Test 1: Health Check del Bridge

```bash
curl http://localhost:8000/health
```

Respuesta esperada:
```json
{
  "status": "ok",
  "service": "Robot Create3 Bridge",
  "openai_configured": true
}
```

### Test 2: Interfaz Web

1. Iniciar bridge: `python robot_bridge.py`
2. Abrir navegador: http://localhost:8000
3. Verificar que carga la interfaz correctamente
4. Clic en "Iniciar Sesión de Voz"
5. Verificar conexión WebSocket exitosa (status cambia a "Conectado")

### Test 3: Comando de Voz Simple

1. Decir: "¿A dónde puedes ir?"
2. Verificar en interfaz:
   - Transcripción aparece en panel izquierdo
   - Se ejecuta `list_available_locations()` (visible en logs)
   - Robot responde con lista de nodos
   - Audio se reproduce en el navegador

### Test 4: Navegación Completa

1. Decir: "Vete al nodo 3 desde el nodo 0"
2. Verificar:
   - Se llama `navigate_robot(destination_id=3, origin_id=0)`
   - Robot confirma inicio de navegación
   - Estado del robot se actualiza en UI (target_node=3)
   - Robot físico se mueve (si está conectado por Bluetooth)

### Test 5: Telemetría en Vivo (NUEVO)

1. Ejecutar suite de pruebas de telemetría:
   ```bash
   python test_telemetry.py
   ```
2. Verificar que pasan 3/3 pruebas:
   - Registro de callback
   - Status incluye posición
   - Simulación de telemetría

3. Prueba en vivo con el bridge:
   - Iniciar navegación hacia cualquier nodo
   - Observar panel "Posición en Vivo" en la interfaz web
   - Verificar que X, Y, θ se actualizan cada ~500ms
   - Verificar animación amarilla en valores al actualizar
   - Al completar, verificar mensaje "✅ Misión completada"

## Debugging

### Logs del Sistema

Todos los logs van a stderr, no stdout (stdout está reservado para protocolo MCP):

- `robot_bridge.py`: Logs en consola donde se ejecuta
- `mcp_server.py`: Logs a stderr (automático cuando se ejecuta como subproceso)
- `robot_service.py`: Logs a stderr con prefijo `[RobotService]`

Para capturar logs:
```bash
python robot_bridge.py 2> bridge.log
```

### Verificar Estado Persistente

```bash
cat last_state.json
# o en Windows:
type last_state.json
```

### Verificación Pre-Demo (Checklist Crítico)

Antes de hacer una demostración en vivo, **VERIFICA ESTOS PUNTOS:**

#### ✅ Audio Funciona
```bash
# 1. Verifica que pydub está instalado
python -c "from pydub import AudioSegment; print('OK')"

# 2. Verifica que ffmpeg está en el PATH
ffmpeg -version

# 3. Si fallas, instala:
pip install pydub
# Y descarga ffmpeg de https://ffmpeg.org/
```

**Síntoma de fallo:** OpenAI no transcribe nada, o transcribe basura.  
**Causa:** Audio WebM no convertido a PCM16.

#### ✅ Lista de Nodos Cargada
```bash
# Iniciar bridge y verificar logs:
python robot_bridge.py

# Busca en la salida:
# "Lista de nodos obtenida para prompt de IA"
```

**Síntoma de fallo:** La IA dice cosas como "No sé qué nodos existen".  
**Causa:** MCP no pudo obtener `list_available_locations()` al inicio.

#### ✅ Robot Conectado por Bluetooth
- Verifica que `config.py` tiene el nombre correcto: `BLUETOOTH_NAME = "iRobot-..."`
- El robot debe estar encendido y en rango
- Primera conexión puede tardar ~30 segundos

#### ✅ OpenAI API Key Válida
```bash
# Verifica que .env existe:
cat .env  # o "type .env" en Windows

# Verifica que la key es válida:
curl https://api.openai.com/v1/models \
  -H "Authorization: Bearer $OPENAI_API_KEY"
```

### Problemas Comunes y Soluciones

#### Error: "ModuleNotFoundError: No module named 'mcp'"
Solución: `pip install mcp`

#### Error: "OPENAI_API_KEY no está configurada"
Causa: Falta archivo `.env` o variable no está definida
Solución: Crear `.env` con `OPENAI_API_KEY=sk-proj-tu-clave`

#### Error: "ERROR: No sé dónde está el robot"
Causa: Primera navegación sin `origin_id` y `current_node` es None
Solución: Proporcionar `origin_id` explícitamente en la primera navegación, o editar `last_state.json`:
```json
{"current_node": 0, "last_message": "Posición inicial"}
```

#### Error: "El robot ya está en movimiento"
Causa: Intentar navegar mientras `state == "NAVIGATING"`
Solución: Llamar primero `emergency_stop()`, luego `navigate_robot()`

#### Error: "Connection refused" al conectar WebSocket
Causa: `robot_bridge.py` no está corriendo
Solución: Iniciar bridge con `python robot_bridge.py`

#### Audio no se escucha en el navegador
Causa: Posible problema de codec o formato de audio
Solución temporal: Verificar que el navegador soporte WebRTC y audio PCM16. Revisar consola del navegador para errores de audio.

#### MCP Server no arranca desde bridge
Causa: Error en `mcp_server.py` o `robot_service.py` al inicializar
Solución: Verificar logs en consola del bridge. Probar ejecutar `python -u mcp_server.py` directamente para ver errores.

## Sistema de Navegación Subyacente

El sistema de navegación física del robot (implementado en `PRM02_P02_EQUIPO1_grafos.py` y `src/potential_fields.py`) utiliza una arquitectura híbrida de planificación global + navegación reactiva:

### Planificación Global

Algoritmo de Dijkstra ejecutado sobre el grafo topológico (`grafos/prueba.py`) para calcular la ruta óptima entre nodos. El grafo contiene:
- 31 nodos con coordenadas espaciales (x, y, theta) en centímetros y grados
- Aristas bidireccionales con pesos que representan coste de navegación
- Nombres descriptivos para cada nodo (inicio, izq_0, med_0, etc.)

**Proceso de planificación:**
1. Usuario solicita navegación: `navigate_robot(destination_id=5, origin_id=0)`
2. Sistema calcula camino mínimo: `graph.Camino_Minimo_Dijkstra(0, 5)` → `[0, 5]`
3. Extrae coordenadas de cada nodo del camino
4. Construye `q_i` (posición inicial), `waypoints` (intermedios), `q_f` (destino final)
5. **NUEVO:** Obtiene heading real del robot antes de usar `q_i` (crítico para transformación correcta)

### Navegación Reactiva con Campos de Potencial Combinados

Campos de potencial combinados implementados en `CombinedPotentialNavigator`:

#### Potencial Atractivo (Linear)
- **Fuerza:** Proporcional a la distancia hacia el waypoint actual
- **Ganancia:** `K_LINEAR = 0.25` (desde config.py)
- **Ganancia angular:** `K_ANGULAR = 3.0` para corrección de orientación rápida
- **Función:** `F_attractive = K_LINEAR * distance_to_goal`

#### Potencial Repulsivo (Evasión de Obstáculos)
- **Modelo:** Basado en **clearance** (distancia libre después del radio del robot)
- **Ganancia:** `K_REPULSIVE = 300.0`
- **Rango de influencia:** `D_INFLUENCE = 100.0 cm` (obstáculos más lejos no generan fuerza)
- **Distancia de seguridad:** `D_SAFE = 20.0 cm` (clearance mínimo recomendado)
- **Modelo físico:** Conversión IR → distancia con compensación por ángulo del sensor
- **7 sensores IR:** Cada sensor tiene factor de normalización específico y ángulo conocido
- **Fuerza repulsiva:** Aumenta drásticamente cuando `clearance < D_SAFE`
  - Clearance crítico (<1cm): Fuerza máxima
  - Clearance insuficiente (<20cm): `F = k_rep * ((1/clearance) - (1/d_safe))²`
  - Clearance suficiente: `F = k_rep * (d_safe/clearance)³ * factor_alcance`

**Problema Identificado:** En espacios reducidos, los nodos pueden estar cerca de paredes (ej: nodo a 25cm, pared a 29cm). El umbral actual hace que el robot evite la pared y no pueda llegar al nodo.

**Solución Propuesta (Pendiente de Implementación):**
- Umbrales adaptativos según proximidad al waypoint:
  - Si `distance_to_goal < 30cm` y `velocity < 10 cm/s`: Reducir `D_INFLUENCE` y `IR_THRESHOLD_DETECT`
  - Esto permite acercarse más a obstáculos cuando está cerca del destino
- Ajuste según velocidad:
  - Velocidad alta: Mantener umbrales actuales (seguridad)
  - Velocidad baja: Reducir umbrales (precisión)

#### Control Dinámico de Velocidad
- **Velocidad máxima:** `V_MAX_CM_S = 38.0 cm/s`
- **Sistema de umbrales escalonados:** Reducción progresiva según nivel de peligro
  - Emergency: `IR_THRESHOLD_EMERGENCY = 700` → Velocidad muy reducida
  - Critical: `IR_THRESHOLD_CRITICAL = 350` → Velocidad reducida
  - Warning: `IR_THRESHOLD_WARNING = 200` → Velocidad moderada
  - Caution: `IR_THRESHOLD_CAUTION = 100` → Velocidad normal con precaución
- **Rampa de aceleración:** `ACCEL_RAMP_CM_S2 = 10.0` (previene cambios bruscos)
- **Zona de desaceleración:** `DECEL_ZONE_CM = 50.0` (comienza a reducir velocidad)

#### Navegación Secuencial
- El robot recorre waypoints intermedios en orden: `q_i → wp1 → wp2 → ... → q_f`
- Cada waypoint se alcanza con tolerancia adaptativa (5-8cm según distancia recorrida)
- Al alcanzar un waypoint, pasa automáticamente al siguiente
- Al alcanzar `q_f`, la misión se completa

#### Transformación de Coordenadas (CRÍTICO)
- **Problema:** El robot usa odometría que empieza en (0,0) después de `reset_navigation()`
- **Solución:** Transformación completa (rotación + traslación) al sistema mundial
- **Heading real:** Se obtiene antes de iniciar navegación (no el theta del grafo)
- **Cálculo:** `odometry_to_world_rotation = desired_heading - reset_heading`
- **Aplicación:** Todas las lecturas de odometría se transforman al sistema mundial durante navegación

### Parámetros Calibrados

Todos los parámetros están en `src/config.py` y fueron calibrados experimentalmente:

- **Geometría del robot:** `ROBOT_RADIUS_CM = 17.095`, `WHEEL_BASE_CM = 23.5`
- **Sensores IR:** 7 sensores con factores de normalización específicos por sensor
- **Ángulos de sensores:** `IR_SENSOR_ANGLES = {-60, -30, 0, 30, 60, 90, 120}` grados
- **Umbrales de seguridad:** `IR_THRESHOLD_EMERGENCY = 700`, `IR_THRESHOLD_CRITICAL = 350`, etc.
- **Tolerancia de llegada:** `TOL_DIST_CM = 3.0` base, adaptativa hasta 8cm según distancia recorrida
- **Período de control:** `CONTROL_DT = 0.05` (20 Hz)

### Integración con MCP

El sistema MCP NO modifica la lógica de navegación existente. Simplemente:
- Expone `CombinedPotentialNavigator` como herramienta MCP
- Usa exactamente los mismos parámetros calibrados
- Mantiene el tipo de potencial fijo en 'linear' (más probado y estable)
- **NUEVO:** Crea navegador con `disable_logging=True` para evitar conflictos de event loop
- **NUEVO:** Ejecuta navegación en `NavigationThread` con event loop dedicado
- **NUEVO:** Obtiene heading real antes de iniciar (no usa theta del grafo)
- Preserva toda la robustez del sistema original

## Correcciones Críticas Implementadas

### ✅ PUNTO CRÍTICO: Conflictos de Event Loop (CORREGIDO)

**El Problema Identificado:**
El SDK de iRobot (`robot.play()`) crea su propio event loop interno. Cuando se ejecuta en modo MCP, hay múltiples event loops ejecutándose simultáneamente:
- FastMCP/uvicorn (event loop del servidor web)
- `robot.play()` (daemon thread con su propio event loop)
- `NavigationThread` (crea su propio event loop para navegación)

Los loggers (`SensorLogger` y `VelocityLogger`) usan `asyncio.Lock` que se vincula al event loop donde se crean, causando errores cuando se usan desde otro event loop:
```
⚠️ Error en logger: <asyncio.locks.Lock object at ...> is bound to a different event loop
```

**La Solución Implementada:**
1. **NavigationThread con event loop dedicado:** La navegación se ejecuta en un thread separado con `asyncio.new_event_loop()` para evitar conflictos
2. **Deshabilitación de loggers en modo MCP:** `CombinedPotentialNavigator` ahora acepta parámetro `disable_logging=True` que evita crear loggers que causan conflictos
3. **Reducción de frecuencia de polling:** Polling de telemetría reducido de 500ms a 1 segundo para reducir carga

**Código relevante:**
```python
# En robot_service.py
self.navigator_instance = CombinedPotentialNavigator(
    self.controller.robot, q_i, waypoints, q_f,
    disable_logging=True  # Evita conflictos de event loop
)

# NavigationThread crea su propio event loop
self.loop = asyncio.new_event_loop()
asyncio.set_event_loop(self.loop)
self.loop.run_until_complete(self._run_mission())
```

**Resultado:** ✅ Navegación funciona correctamente sin errores de event loop

### ✅ PUNTO CRÍTICO: Navegación Consecutiva con Heading Correcto (CORREGIDO)

**El Problema Identificado:**
Cuando el robot navega consecutivamente (ej: 0→5, luego 5→2), el sistema usaba el theta almacenado en el grafo (ej: 0°) en lugar del heading real del robot cuando llegó al nodo 5 (ej: 45°). Esto causaba transformación de coordenadas incorrecta y navegación errónea.

**La Solución Implementada:**
1. **Guardado de heading en last_state.json:** Al finalizar cada navegación, se guarda el heading real del robot
2. **Obtención de heading real antes de navegar:** En `_run_mission()`, se lee el heading real del robot antes de iniciar navegación
3. **Prioridad de heading:** 
   - Primero: Heading real del robot (si disponible)
   - Segundo: `last_heading` guardado en disco (si existe)
   - Tercero: Theta del grafo (valor por defecto)

**Código relevante:**
```python
# En robot_service.py _run_mission()
try:
    pos = await self.controller.robot.get_position()
    if pos is not None:
        actual_theta = pos.heading  # Heading REAL del robot
    elif self.controller.last_heading is not None:
        actual_theta = self.controller.last_heading  # Heading guardado
except:
    actual_theta = q_i_node["theta"]  # Theta del grafo (fallback)

# Al finalizar navegación
final_pos = await self.controller.robot.get_position()
if final_pos is not None:
    self.controller.last_heading = final_pos.heading
    self.controller._write_state_to_disk()
```

**Resultado:** ✅ Navegación consecutiva funciona correctamente con transformación de coordenadas precisa

## Revisión Crítica de Arquitectura

### ✅ PUNTO 1: "Fire and Forget" Disconnection (CORREGIDO)

**El Problema Identificado:**
La herramienta `navigate_robot` retorna inmediatamente con "RECIBIDO: Iniciando navegación...", lo que hace que OpenAI piense que la misión está completa cuando apenas empieza. Si el robot falla 10 segundos después (deadlock, trampa local), OpenAI no se entera a menos que el usuario pregunte explícitamente.

**La Solución Implementada:**
- En `robot_bridge.py`, el bucle `telemetry_polling()` detecta cambios de estado
- Cuando `mission_completed` cambia de `False` → `True`, inyectamos un mensaje en la conversación de OpenAI:
  ```python
  await self.openai_ws.send(json.dumps({
      "type": "conversation.item.create",
      "item": {
          "type": "message",
          "role": "user",
          "content": [{
              "type": "input_text",
              "text": "[SYSTEM ALERT] Navegación completada exitosamente. ..."
          }]
      }
  }))
  await self.openai_ws.send(json.dumps({"type": "response.create"}))
  ```
- Lo mismo para errores: inyectamos `[SYSTEM ERROR] La navegación falló. ...`

**Resultado:** OpenAI ahora se entera automáticamente cuando las misiones terminan (éxito o fallo) sin necesidad de que el usuario pregunte.

### ✅ PUNTO 2: Race Conditions en State Management (CORREGIDO)

**El Problema Identificado:**
`last_state.json` se escribe desde múltiples funciones async sin protección. Si `emergency_stop()` y `_run_mission()` intentan escribir simultáneamente, podríamos corromper el archivo o tener dirty reads.

**La Solución Implementada:**
- Agregado `self.state_lock = asyncio.Lock()` en `RobotController.__init__()`
- Creadas dos versiones de `save_state()`:
  - `save_state()`: Versión sync para código síncrono (ej: `start_navigation()`)
  - `save_state_async()`: Versión async con lock para código async
- Todas las operaciones críticas ahora usan lock:
  ```python
  async with self.state_lock:
      self.state = "IDLE"
      self.current_node = dest_node
      # ... operaciones atómicas ...
  
  await self.save_state_async()  # Escritura protegida
  ```
- `get_status()` ahora también usa lock para lecturas atómicas
- File I/O se ejecuta en thread pool vía `run_in_executor()` para no bloquear el event loop

**Resultado:** Todas las operaciones de estado son ahora atómicas y thread-safe.

### ⚠️ PUNTO 3: CombinedPotentialNavigator Initialization (YA CORRECTO)

**La Preocupación:**
Re-instanciar `CombinedPotentialNavigator` en cada misión podría causar overhead de reconexión Bluetooth o recalibración de sensores.

**Análisis del Código:**
Revisando `PRM02_P02_EQUIPO1_grafos.py` líneas 361-391, el `__init__` solo hace:
- Asignación de referencias (`self.robot = robot`)
- Creación de loggers (ligero)
- **NO** hace conexión Bluetooth (recibe objeto `robot` ya conectado)
- **NO** hace calibración de sensores

**Conclusión:** La implementación actual es correcta. No hay overhead significativo porque:
1. El objeto `robot` se reutiliza (conexión persistente en `robot_service.py`)
2. El `__init__` es extremadamente ligero (<1ms)
3. Cada misión necesita waypoints diferentes, re-instanciar es apropiado

**Validación Sugerida:** Medir tiempo entre "RECIBIDO" y primer movimiento de ruedas. Debe ser <500ms (tiempo de planificación Dijkstra, no inicialización).

### 🔴 Problema #1: Audio WebM → PCM16 (CORREGIDO)

**El Problema:**
Los navegadores envían audio en formato WebM/Opus por defecto. La API Realtime de OpenAI requiere PCM16 (raw audio) a 24kHz mono. Enviar WebM directamente causa que OpenAI reciba ruido estático y no transcriba nada.

**La Solución Implementada:**
- Función `convert_webm_to_pcm16()` en `robot_bridge.py`
- Usa `pydub` + `ffmpeg` para decodificar en tiempo real
- Conversión automática: WebM/Opus → PCM16 mono 24kHz
- Fallback si la conversión falla (con advertencia en logs)

**Código relevante (robot_bridge.py líneas 90-128):**
```python
def convert_webm_to_pcm16(webm_data: bytes) -> bytes:
    audio = AudioSegment.from_file(io.BytesIO(webm_data), format="webm")
    audio = audio.set_channels(1)  # Mono
    audio = audio.set_frame_rate(24000)  # 24kHz
    audio = audio.set_sample_width(2)  # 16-bit
    return audio.raw_data
```

### 🟡 Problema #2: Alucinaciones de IDs de Nodos (CORREGIDO)

**El Problema:**
La IA puede inventar IDs de nodos inexistentes porque no conoce a priori qué lugares existen en el mapa. Esto genera errores al intentar navegar.

**La Solución Implementada:**
- Al conectar a MCP, se llama `list_available_locations()` automáticamente
- La lista completa de nodos se inyecta dinámicamente en `SYSTEM_INSTRUCTIONS`
- La IA ahora conoce desde el inicio: ID 0: inicio, ID 1: izq_0, etc.
- Instrucción explícita: "IMPORTANTE: Estos son los ÚNICOS nodos válidos. NO inventes IDs"

**Código relevante (robot_bridge.py líneas 215-228):**
```python
# Obtener lista de nodos para inyectar en el prompt
result = await self.mcp_session.call_tool("list_available_locations", {})
self.node_list = content_item.text

# Más tarde, al configurar OpenAI:
system_instructions = get_system_instructions(self.node_list)
# Esto inyecta: "MAPA DEL ROBOT (Nodos disponibles): ..."
```

**Resultado:**
- ✅ La IA no necesita llamar `list_available_locations()` cada vez
- ✅ Reduce latencia (una llamada menos por conversación)
- ✅ Elimina alucinaciones de nombres/IDs inexistentes

## Implementación Técnica del Bridge Server

### Arquitectura del Bridge (robot_bridge.py)

El bridge server implementa tres conexiones simultáneas:

1. **Conexión con Browser (WebSocket)**: Recibe audio del micrófono del usuario y envía respuestas de voz de la IA
2. **Conexión con OpenAI Realtime API (WebSocket)**: Envía audio del usuario, recibe transcripciones y llamadas a herramientas
3. **Conexión con MCP Server (stdio)**: Ejecuta `mcp_server.py` como subproceso y comunica mediante JSON-RPC 2.0

### Clase BridgeSession

La clase `BridgeSession` gestiona el ciclo de vida completo de una sesión:

- `connect_to_mcp()`: Inicia `mcp_server.py` como subproceso usando `mcp.client.stdio`, descubre herramientas disponibles y las convierte al formato OpenAI
- `connect_to_openai()`: Establece conexión WebSocket con OpenAI Realtime API, configura la sesión con herramientas MCP, instrucciones del sistema y parámetros de audio
- `communication_loop()`: Ejecuta dos tareas en paralelo:
  - `browser_to_openai()`: Recibe audio del browser, convierte a base64 y reenvía a OpenAI
  - `openai_to_browser_and_mcp()`: Procesa eventos de OpenAI (transcripciones, audio de respuesta, llamadas a herramientas) y los enruta apropiadamente
- `handle_tool_call()`: Ejecuta herramientas MCP cuando OpenAI las solicita, formatea resultados y los retorna a OpenAI para generar respuesta de voz

### Manejo de Audio

El bridge maneja audio en formato PCM16 de 24kHz:
- Browser envía audio WebM que se convierte a base64 para OpenAI
- OpenAI envía audio PCM16 en base64 que se decodifica y reenvía al browser
- La conversión de formatos se maneja automáticamente por las librerías

### Configuración de OpenAI Realtime

El bridge configura la sesión de OpenAI con:
- Modalities: text y audio habilitados
- Voice: "alloy" (puede cambiarse en `SYSTEM_INSTRUCTIONS`)
- Input/Output audio format: PCM16
- Transcription model: Whisper-1
- Turn detection: Server VAD (Voice Activity Detection) con umbral 0.5
- Tool choice: "auto" (OpenAI decide cuándo llamar herramientas)
- Temperature: 0.8 para respuestas más naturales

### Instrucciones del Sistema

Las instrucciones del sistema (`SYSTEM_INSTRUCTIONS`) guían el comportamiento de la IA:

**REGLAS CRÍTICAS PARA NAVEGACIÓN:**
1. SIEMPRE necesitas DOS valores para navegar: `origin_id` (de dónde sale) y `destination_id` (a dónde va)
2. Si el usuario NO especifica el nodo origen EXPLÍCITAMENTE con un número:
   - PRIMERO llama a `get_robot_status()` para obtener el `current_node`
   - LUEGO usa ese `current_node` como `origin_id`
3. Si el usuario dice "el último", "donde está", "donde quedó", "su posición actual":
   - DEBES llamar a `get_robot_status()` primero
   - Usa el `current_node` de la respuesta como `origin_id`
4. NUNCA llames a `navigate_robot()` sin tener AMBOS valores confirmados
5. Si el usuario dice "del nodo X al Y" → `origin_id=X`, `destination_id=Y`
6. Si el usuario solo dice "ve al nodo Y" sin especificar origen → llama `get_robot_status()` primero

**Otras instrucciones:**
- Explican las 4 herramientas MCP disponibles
- Instruyen a la IA a usar `list_available_locations()` primero cuando el usuario pregunta por lugares
- Enfatizan el uso de IDs numéricos exactos (no inventar números)
- Instruyen a llamar `emergency_stop()` inmediatamente si el usuario dice "para" o "detente"

## Detalles de Implementación

### Gestión de Estado Persistente

El archivo `last_state.json` se actualiza automáticamente cuando:
- Una misión de navegación termina exitosamente (`current_node` se actualiza al destino)
- Se ejecuta `emergency_stop()` (estado se resetea a IDLE)

Formato del archivo:
```json
{
  "current_node": 3,
  "last_heading": 45.2,
  "last_message": "Llegada exitosa al Nodo 3."
}
```

**NUEVO:** El campo `last_heading` es crítico para navegación consecutiva. Cuando el robot llega a un nodo desde cualquier dirección, su heading real puede ser diferente al theta almacenado en el grafo. El sistema ahora:
1. Guarda el heading real al finalizar cada navegación
2. Usa este heading al iniciar la siguiente navegación (en lugar del theta del grafo)
3. Esto asegura que la transformación de coordenadas sea correcta en navegaciones consecutivas

Si el servidor crashea, el último estado conocido se mantiene y se carga al reiniciar.

### Conexión Bluetooth Persistente

La conexión al robot Create3 se establece una sola vez cuando se crea la instancia de `RobotController` en `robot_service.py`. La conexión se mantiene durante toda la vida del servidor MCP, evitando overhead de reconexión en cada comando.

### Navegación Asíncrona con Event Loop Dedicado

**ARQUITECTURA ACTUALIZADA:** Las misiones de navegación se ejecutan en `NavigationThread` (threading.Thread) con su propio event loop de asyncio para evitar conflictos con FastMCP/uvicorn:

- **NavigationThread:** Crea su propio `asyncio.new_event_loop()` para ejecutar `navigator.navigate()`
- **Deshabilitación de loggers:** `CombinedPotentialNavigator` se crea con `disable_logging=True` para evitar conflictos de `asyncio.Lock` entre event loops
- **Obtención de heading real:** Antes de iniciar navegación, se lee el heading real del robot (no el theta del grafo)
- El servidor MCP no se bloquea durante navegaciones largas
- Se pueden recibir otros comandos (como `emergency_stop()`)
- El bridge puede seguir procesando llamadas de herramientas
- **Problema resuelto:** Evita errores de "Lock bound to different event loop" que causaban navegación incorrecta

### Manejo de Errores

El sistema maneja errores en múltiples niveles:
- Validación de parámetros antes de iniciar navegación
- Manejo de excepciones en bucles de control
- Cancelación limpia de tareas asíncronas
- Logging detallado a stderr para debugging

## Métricas del Proyecto

Archivos nuevos creados:
- `robot_bridge.py`: 490 líneas (servidor bridge FastAPI + telemetría)
- `static/index.html`: 650 líneas (interfaz web + posición en vivo)
- `robot_service.py`: 430 líneas (controlador del robot + telemetría)
- `mcp_server.py`: 204 líneas (servidor MCP)
- `test_mcp_server.py`: 296 líneas (suite de pruebas base)
- `test_telemetry.py`: 190 líneas (suite de pruebas telemetría)
- `last_state.json`: Estado persistente

Archivos existentes (sin modificaciones):
- `PRM02_P02_EQUIPO1_grafos.py`: 1057 líneas
- `src/potential_fields.py`: 1557 líneas
- `src/config.py`: 461 líneas
- `grafos/prueba.py`: 408 líneas

Total del sistema: ~5800 líneas de código Python/HTML/JS

Cobertura de pruebas:
- Suite base: 6/6 pruebas pasadas (imports, grafo, Dijkstra, controlador, nombres, persistencia)
- Suite telemetría: 3/3 pruebas pasadas (callback, status, simulación)

## Notas Técnicas

- Conexión Persistente: El robot mantiene conexión Bluetooth durante toda la vida del servidor MCP
- Navegación Async: Las misiones se ejecutan en background, el servidor no se bloquea
- Estado Seguro: Si el servidor crashea, el último estado conocido se mantiene en JSON
- Tipo de Potencial: Fijo en 'linear' por decisión de diseño (más probado y estable)
- **Audio PCM16:** Conversión automática WebM → PCM16 con pydub + ffmpeg (crítico para OpenAI)
- **Prompt Inyectado:** Lista de nodos pre-cargada en SYSTEM_INSTRUCTIONS para reducir alucinaciones
- **Telemetría en Vivo:** Sistema de polling a 1 Hz desde bridge (reducido de 2 Hz), 5 Hz desde robot durante navegación
- **Detección Automática:** El bridge detecta finalización de misiones sin intervención de OpenAI
- **Eventos Asíncronos:** Tres bucles paralelos (audio, OpenAI, telemetría) ejecutándose simultáneamente
- **Event Loop Dedicado:** NavigationThread con su propio event loop evita conflictos con FastMCP/uvicorn
- **Logging Deshabilitado:** `disable_logging=True` en modo MCP evita conflictos de asyncio.Lock
- **Heading Persistente:** `last_heading` guardado en `last_state.json` para navegación consecutiva correcta
- **Instrucciones Mejoradas:** Sistema de IA ahora requiere `origin_id` explícito, llama `get_robot_status()` si no lo conoce
- Windows Compatible: Probado en Windows 10/11 con Python 3.x
- Buffering: El bridge ejecuta `mcp_server.py` con flag `-u` automáticamente para evitar problemas de buffering en Windows

### Mejoras Futuras Opcionales

**1. Telemetría con Server-Sent Events (MCP Nativo):**
- Actualmente: Polling cada 1 segundo (reducido de 500ms para evitar conflictos)
- Mejora: Usar Context de FastMCP para enviar actualizaciones proactivas
- Beneficio: Latencia más baja (<50ms), menos overhead de red
- Trade-off: Más complejidad, MCP SSE tiene limitaciones para eventos push

**2. Umbrales Adaptativos de Obstáculos (PENDIENTE DE IMPLEMENTACIÓN):**
- **Problema Actual:** En espacios reducidos, los nodos pueden estar cerca de paredes (ej: nodo a 25cm, pared a 29cm). El umbral actual (`D_INFLUENCE = 100cm`, `IR_THRESHOLD_DETECT`) hace que el robot evite la pared y no pueda llegar al nodo.
- **Solución Propuesta:** Sistema de umbrales adaptativos según:
  - **Proximidad al waypoint:** Si `distance_to_goal < 30cm` → Reducir `D_INFLUENCE` progresivamente (ej: 100cm → 50cm → 30cm)
  - **Velocidad del robot:** Si `velocity < 10 cm/s` → Reducir `IR_THRESHOLD_DETECT` (permitir detección más cercana)
  - **Combinación:** Cuando ambas condiciones se cumplen, el robot puede acercarse más a obstáculos para alcanzar el nodo
- **Beneficio:** Permite llegar a nodos cerca de paredes sin evitar obstáculos innecesariamente
- **Implementación Sugerida:** Modificar `repulsive_force()` en `src/potential_fields.py` para calcular umbrales dinámicos basados en `distance_to_goal` y `current_velocity`

**2. Visualización Gráfica del Mapa:**
- Canvas HTML5 para dibujar trayectoria en tiempo real
- Indicadores visuales de sensores IR (distancia a obstáculos)
- Animación del robot moviéndose en el mapa 2D

**3. Audio Bidireccional Optimizado:**
- Actualmente: Conversión completa por chunk (funcional pero CPU-intensivo)
- Mejora: Streaming pipeline con buffer circular
- Beneficio: Menor latencia, menor uso de CPU

## Autores

Alan Ariel Salazar  
Yago Ramos Sánchez

Universidad Intercontinental de la Empresa (UIE)  
Curso: Robots Autónomos  
Profesor: Eladio Dapena  
Fecha: Noviembre 2025

