<!-- c2ccf0ce-2c7c-490c-8987-16287b0c4ea9 820ea508-afb5-4144-abcc-f75263ade6ed -->
# Integración MCP para Robot Create3

## 1. Crear robot_service.py

Archivo nuevo en raíz del proyecto que encapsula la lógica de control del robot:

**Imports necesarios**:

- `src.config` (ya existe)
- `grafos.prueba.cargar_grafo_desde_json` (ya existe)
- `irobot_edu_sdk.backend.bluetooth.Bluetooth` y `irobot_edu_sdk.robots.Create3`
- La clase `CombinedPotentialNavigator` desde `PRM02_P02_EQUIPO1_grafos` (líneas 330-831)

**Clase RobotController**:

Atributos principales:

- `self.graph`: cargado una vez con `cargar_grafo_desde_json()`
- `self.robot`: conexión persistente `Create3(Bluetooth(config.BLUETOOTH_NAME))`
- `self.current_node: int | None`: persistido en `last_state.json`
- `self.state`: `"IDLE" | "NAVIGATING" | "ERROR"`
- `self.target_node: int | None`
- `self.last_message: str`
- `self.mission_task: asyncio.Task | None`: para navegación en background
- `self.navigator_instance`: referencia al `CombinedPotentialNavigator` activo

Métodos clave:

- `load_state()` / `save_state()`: JSON con `{"current_node": int}`
- `get_node_names() -> dict[int, str]`: retorna `{id: graph.nombres[id]}`
- `ensure_robot_connected()`: crea `Create3` si no existe (conexión persistente)
- `start_navigation(destination_id: int, origin_id: int | None = None) -> str`:
  - Validación: si `NAVIGATING` → error
  - Determina origen: `origin_id` o `self.current_node` (si ambos `None` → error)
  - Valida `0 <= nodos < graph.V`
  - Lanza `asyncio.create_task(self._run_mission(origin, dest))`
  - Retorna inmediatamente mensaje confirmación
- `_run_mission(origin_node: int, dest_node: int)` (async interno):
  - Llama `graph.Camino_Minimo_Dijkstra(origin, dest)` → obtiene `path_indices, total_cost`
  - Construye `q_i`, `waypoints`, `q_f` desde `graph.coords` (igual que líneas 948-957 de PRM02_P02_EQUIPO1_grafos.py)
  - Crea `CombinedPotentialNavigator(robot, q_i, waypoints, q_f, potential_type='linear', k_rep=config.K_REPULSIVE, d_influence=config.D_INFLUENCE)`
  - Ejecuta `success = await navigator.navigate()`
  - Si `success`: actualiza `self.current_node = dest`, `save_state()`, `self.state = "IDLE"`
  - Si fallo o cancelación: `self.state = "ERROR"`, mensaje apropiado
- `emergency_stop() -> str` (async):
  - Cancela `self.mission_task` si existe
  - Llama `await robot.set_wheel_speeds(0, 0)` y `await robot.stop()`
  - Ajusta `self.state = "IDLE"`, retorna mensaje
- `get_status() -> dict`: retorna `{"state", "current_node", "target_node", "battery_level": "Unknown", "last_message"}`

**Adaptaciones específicas al código existente**:

- NO modificar la lógica de `CombinedPotentialNavigator` (mantener tal cual líneas 330-831)
- Usar exactamente los mismos parámetros: `config.K_REPULSIVE`, `config.D_INFLUENCE`, `config.TOL_DIST_CM`, etc.
- La construcción de `q_i`, `waypoints`, `q_f` debe ser idéntica a líneas 948-957
- Manejar `robot.play()` correctamente: el SDK Create3 requiere contexto de ejecución. Como el navegador ya está diseñado para funcionar dentro de un callback `@robot.when_play`, adaptar para que funcione en contexto asyncio existente (el servidor MCP ya tiene event loop)

## 2. Crear mcp_server.py

Archivo nuevo en raíz que expone herramientas MCP:

**Setup**:

```python
from mcp.server.fastmcp import FastMCP
from robot_service import RobotController
import sys

controller = RobotController()  # instancia única persistente
mcp = FastMCP("Create3 Topological Navigator")
```

**Herramientas MCP**:

1. `list_available_locations() -> str`:

   - Llama `controller.get_node_names()`
   - Retorna string formateado: `"0: NombreNodo1\n1: NombreNodo2\n..."`

2. `navigate_robot(destination_id: int, origin_id: int | None = None) -> str`:

   - Llama `controller.start_navigation(destination_id, origin_id)`
   - Retorna inmediatamente (no bloquea)
   - Docstring claro explicando que es async

3. `emergency_stop() -> str` (async):

   - Llama `await controller.emergency_stop()`
   - Retorna mensaje de confirmación

4. `get_robot_status() -> dict`:

   - Llama `controller.get_status()`
   - Retorna dict estructurado

**Ejecución**:

```python
if __name__ == "__main__":
    print("Iniciando Servidor MCP iRobot Create3...", file=sys.stderr)
    mcp.run()  # stdio mode
```

## 3. Ajustes a PRM02_P02_EQUIPO1_grafos.py (si necesario)

Verificar que `CombinedPotentialNavigator` puede ser importado limpiamente. Si hay dependencias circulares o código que solo funciona en `if __name__ == "__main__"`, refactorizar mínimamente para permitir import.

## 4. Crear last_state.json (placeholder)

Archivo inicial vacío o con estructura: `{"current_node": null, "last_message": "Sistema inicializado"}`

## 5. Dependencias

Verificar instalación de `mcp`:

```bash
pip install mcp
```

## Consideraciones Críticas

**Manejo del SDK Create3 en background**:

- El SDK iRobot usa decorador `@robot.when_play` que ejecuta en su propio event loop
- En `_run_mission`, necesitamos ejecutar la navegación sin bloquear
- Solución: llamar directamente a métodos del robot en el event loop asyncio existente (FastMCP ya proporciona uno)
- El `navigator.navigate()` es async y llama métodos del robot, debería funcionar directamente

**Windows y stdio**:

- El servidor se ejecutará con `python -u mcp_server.py` (sin búfer)
- Todos los logs deben ir a `sys.stderr`, no `stdout` (stdout es para protocolo MCP)
- Añadir `file=sys.stderr` a todos los `print()` en `robot_service.py`

**Estado persistente**:

- `last_state.json` se guarda solo cuando una misión termina exitosamente
- Si el servidor se cierra inesperadamente, mantiene el último estado conocido

**Tipo de potencial fijo**:

- Siempre usar `potential_type='linear'` (decisión del usuario)
- Parámetros `k_rep` y `d_influence` desde `config.py` (valores ya calibrados)

### To-dos

- [ ] Crear robot_service.py con clase RobotController completa: carga grafo, gestión estado persistente, métodos de navegación async
- [ ] Crear mcp_server.py con FastMCP y las 4 herramientas: list_locations, navigate_robot, emergency_stop, get_status
- [ ] Verificar que CombinedPotentialNavigator puede importarse desde PRM02_P02_EQUIPO1_grafos.py sin ejecutar main()
- [ ] Probar inicio del servidor con python -u mcp_server.py y verificar carga correcta del grafo