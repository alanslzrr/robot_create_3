# Sistema de Navegación y Mapeo para iRobot Create3 — Documentación técnica

## 1. Propósito

Navegar de forma autónoma entre puntos espaciales definidos por el usuario (nodos), manteniendo rumbo y evitando obstáculos con sensores IR, garantizando seguridad y trazabilidad completa (telemetría, intentos, segmentos).

---

## 2. Arquitectura lógica

* **Capa de interacción**: `nav_menu.py` (GUI) y `teleop_mark_nodes.py` (teleoperación y marcado).
* **Capa de navegación reactiva**: `core/ir_avoid.py` (Bug2 formalizado con IR).
* **Capa de seguridad/telemetría**: `core/safety.py`, `core/telemetry.py`.
* **Capa de utilidades y persistencia**: `nodes_io.py`, `core/undock.py`, `core/config_validator.py`.
* **Capa de análisis y visualización**: `visualize_nodes.py`.
* **Configuración**: `config.yaml`.
* **Datos**: `nodes/` (JSONL y CSV de logs).

Cada capa expone funciones bien definidas y sincrónicas/asíncronas que se combinan en `nav_menu.py` para cumplir el objetivo: seleccionar origen y destino, ejecutar navegación IR robusta, registrar y auditar.

---

## 3. Componentes y responsabilidades

### 3.1 Interacción

* **`teleop_mark_nodes.py`**

  * Teleop con flechas/WASD y seguridad manual.
  * Marca nodos (`G`) y segmenta movimiento entre nodos en segmentos cinemáticos.
  * Genera:

    * `nodes/nodes.jsonl`: nodos `{id,name,x,y,theta,ts}`.
    * `nodes/edges.jsonl`: aristas con `segments` y agregados.
    * CSV por arista: `nodes/logs/edge_*.csv`.
  * Usa: `nodes_io.py`, `core/undock.py`, `core/safety.py`, `core/telemetry.py`.
  * Objetivo: construir el grafo de navegación con métricas reales (odometría vs plan).

* **`nav_menu.py`**

  * GUI (Tkinter) para navegación autónoma.
  * Define origen: `Undock` (dock actual) o `Start Nodo` (pose conocida) y confirma.
  * Navegación a nodos por **ID** o **nombre**.
  * “Parar” cancela la tarea de navegación en curso con parada segura.
  * Indicadores: conexión y estado de `SafetyMonitorV2`.
  * Registro de intentos: `nodes/logs/nav_attempts.csv`.
  * Integra: `IRAvoidNavigator` para navegación y evita usar `navigate_to(0,0)` directo; “Ir a origen” usa IR Avoid a `(0,0)`.

### 3.2 Navegación

* **`core/ir_avoid.py`**

  * `IRAvoidNavigator`: navegación reactiva basada en **Bug2 formal** con IR.
  * Estados:

    * `SEEK`: corrección de rumbo a objetivo por odometría, control proporcional angular.
    * `WALL_FOLLOW`: bordeo con control proporcional a distancia lateral (IR).
    * Transiciones **HIT/LEAVE** con M-line, proyección escalar sobre la M-line y ventana refractaria post-salida para evitar reenganche inmediato.
  * Sensores IR:

    * Frontal `[3]`, laterales izq. `[0,1]`, der. `[5,6]` (configurable).
    * Filtro IIR y **histéresis** de umbral frontal para estabilidad.
  * Stuck handling: si no hay progreso proyectado en M-line, ejecuta back-off + giro correctivo.
  * **Arco de despegue** al salir de pared (LEAVE): pequeño avance curvado alejándose de la pared para reducir lecturas frontales y evitar reenganche.
  * Respeta `SafetyMonitorV2` y actualiza `TelemetryLogger`.

### 3.3 Seguridad y telemetría

* **`core/safety.py` — `SafetyMonitorV2`**

  * Sondeo periódico de IR/bumpers/cliff.
  * Freno momentáneo, LED rojo y bandera `halted`.
  * `clear_halt()` para override explícito.
* **`core/telemetry.py` — `TelemetryLogger`**

  * ~10 Hz: `pose`, `cmd_vl/vr`, `acc_rms`, `ir_max`, `bumps`, `battery`.
  * La navegación actualiza `update_command(vl,vr)` antes de loguear.

### 3.4 Utilidades/persistencia

* **`nodes_io.py`**

  * Carga/guarda JSONL, índices por id/nombre, vecinos y rutas faltantes.
  * Agrega métricas por arista (`aggregate_edge`) y registra CSV por segmentos.
  * `log_nav_attempt(...)` para `nav_attempts.csv`.
* **`core/undock.py`**

  * Secuencia estándar para salir del dock: retroceso controlado + giro + `reset_navigation()`.
* **`core/config_validator.py`**

  * Valida `config.yaml` (robot/motion/safety/telemetry/undock) y rangos.

### 3.5 Visualización

* **`visualize_nodes.py`**

  * Grafo con orientación, aristas coloreadas por calidad, etiquetas.
  * Estadísticas y análisis de una arista específica.
  * Export de imagen.

---

## 4. Datos y formatos

* `nodes/nodes.jsonl`: línea por nodo `{id,name,x,y,theta,ts,source}`.
* `nodes/edges.jsonl`: línea por arista `{from,to,segments,agg,quality,ts}`.
* `nodes/logs/edge_*.csv`: por segmento `state,t,dist_cm,deg,odom_dist_cm,odom_deg,err_*`.
* `nodes/logs/nav_attempts.csv`: intento `{ts,target,plan_x/plan_y,pose inicio/fin,err_dist_cm,err_deg,origin_type,origin_id}`.
* `nodes/logs/telemetry_*.csv`: muestreo periódico de sensores y comandos.
* `nodes/VERSION`: versión del formato del mapa.

---

## 5. Configuración operativa (`config.yaml`)

Secciones mínimas:

* `robot.name`: nombre Bluetooth.
* `motion`: `vel_default_cm_s`, `giro_default_cm_s`, `track_width_cm`, `linear_scale`, `angular_scale`.
* `safety`: `ir_threshold`, `safety_period_s`, `enable_auto_brake`.
* `telemetry`: `period_s`, `log_dir`.
* `undock`: `back_cm`, `back_speed`, `turn_deg`, `turn_dir`.
* `avoidance`:

  * Umbrales: `ir_obs_threshold` (p.e. 120), `ir_dir_threshold` (p.e. 200).
  * Índices: `front_idx`, `left_idx`, `right_idx`.
  * Velocidades: `cruise_cm_s`, `turn_cm_s`.
  * Control: `goal_kp`, `wall_kp`, `goal_tolerance_cm`, `reacquire_deg`.
  * Robustez: `timeout_s`, `progress_eps_cm`, `progress_dt_s`.

Justificación de valores:

* 120/200 y mapeos de índices IR se alinean con mediciones físicas y ejemplos de laboratorio.
* `goal_kp` pequeño para evitar sobreviraje; `wall_kp` bajo para bordeo estable.
* `reacquire_deg` limita reenganche a rumbos viables.

---

## 6. Algoritmo de navegación (IRAvoidNavigator)

### 6.1 Definición de rumbo al objetivo

* Modo `SEEK`:

  * Se define el ángulo hacia el objetivo a partir de odometría (`get_position()`).
  * Control proporcional sobre el error angular y cálculo de la consigna diferencial de ruedas con saturación uniforme.

$$
\theta_{goal} = \operatorname{atan2}(y_g - y,\, x_g - x)\\
e_\theta = \operatorname{wrap}_{(-180,180]}\!\big(\theta_{goal} - \theta\big)\\
\Delta = K_{p}^{goal}\, e_\theta\\
v_l = v_{base} - \Delta,\quad v_r = v_{base} + \Delta\\
\max(|v_l|,|v_r|) \le V_{\max}
$$

Donde:
$$
\begin{aligned}
x,\ y &:\ \text{posición actual del robot (cm)} \\
x_g,\ y_g &:\ \text{posición objetivo (cm)} \\
\theta,\ \theta_{goal} &:\ \text{orientación actual y rumbo al objetivo (grados)} \\
e_\theta &:\ \text{error angular envuelto a }(-180,180] \text{ (grados)} \\
K_{p}^{goal} &:\ \text{ganancia proporcional de rumbo} \\
v_{base} &:\ \text{velocidad base (cm/s)} \\
v_l,\ v_r &:\ \text{velocidades de ruedas (cm/s)} \\
V_{\max} &:\ \text{cota de saturación por rueda (cm/s)}
\end{aligned}
$$

**Por qué**: se corrige rumbo continuamente sin perder dirección hacia el punto espacial.

### 6.2 Detección de obstáculo y selección de lado

* Si el IR frontal filtrado supera `ir_obs_threshold` (con histéresis), se conmuta a `WALL_FOLLOW`.
* Se elige el lado más libre comparando valores laterales máximos (menor = más libre).

**Por qué**: evita embestir y reduce giros hacia zonas congestionadas.

En la práctica usamos histéresis frontal; además, el lado de bordeo se elige por el lateral con menor lectura máxima.

$$
\text{front\_blocked} = \begin{cases}
1, & f > T_{on} \\
0, & f < T_{off} = 0.7\,T_{on}
\end{cases}
\qquad
s^{\*} = \operatorname*{arg\,min}_{s\in\{L,R\}}\big\{\max(\mathrm{IR}_s)\big\}
$$

**Implementación mejorada**: Se usa un margen del 10% para evitar oscilaciones en la selección de lado cuando las lecturas laterales son similares.

Donde:
$$
\begin{aligned}
f &:\ \text{lectura IR frontal filtrada} \\
T_{on},\ T_{off} &:\ \text{umbrales de histéresis (}T_{off}=0.7\,T_{on}\text{)} \\
\mathrm{IR}_s &:\ \text{lectura lateral máxima del lado } s\in\{L,R\} \\
\text{front\_blocked} &:\ \text{indicador binario de bloqueo frontal} \\
s^{\*} &:\ \text{lado elegido para bordeo}
\end{aligned}
$$

### 6.3 Bordeo de pared (`WALL_FOLLOW`)

* Control proporcional lateral: error = lectura_lateral − 0.8·`ir_dir_threshold`.
* Ajuste diferencial de ruedas para mantener distancia de bordeo.
* Si frontal vuelve a cerrarse, prioridad al giro “hacia afuera”.

**Por qué**: permite rodear obstáculos conservando avance.

Control lateral y priorización de giro hacia afuera cuando el frontal vuelve a cerrarse:

$$
e_{lat} = \mathrm{IR}_{side} - 0.8\,T_{dir},\quad
\delta = K_{p}^{wall}\, e_{lat}\\
\text{si side=left: } v_l = v_{base} - \delta,\ v_r = v_{base} + \delta\\
\text{si side=right: } v_l = v_{base} + \delta,\ v_r = v_{base} - \delta\\
\text{si } f > T_{on}:\ (v_l, v_r) \leftarrow (v_{base} \pm V_{turn})
$$

Donde:
$$
\begin{aligned}
\mathrm{IR}_{side} &:\ \text{lectura IR del lado de bordeo} \\
T_{dir} &:\ \text{umbral lateral (distancia deseada)} \\
K_{p}^{wall} &:\ \text{ganancia de bordeo} \\
\delta &:\ \text{corrección diferencial} \\
V_{turn} &:\ \text{magnitud del giro} \\
v_{base},\ v_l,\ v_r &:\ \text{velocidades (cm/s)}
\end{aligned}
$$

### 6.4 Condición formal de reenganche (Bug2)

Se vuelve a `SEEK` bajo condiciones **geométricas**, sin exigir frontal despejado:

1. Distancia a la M-line `< ε` (≈ 3 cm).
2. Más cerca del objetivo que en el punto de impacto (**`hit`**).
3. Proyección escalar sobre la M-line **ha avanzado** más allá del `hit` (evita salidas prematuras).
4. Error angular razonable hacia el objetivo. Umbral **adaptativo**: `max(reacquire_deg, min(30°, 0.2°/cm · dist))`.
5. Tras disparar LEAVE, se ejecuta un **arco de despegue** corto alejándose de la pared y se aplica una **ventana refractaria** (~0.4 s) para impedir reenganche inmediato por ruido.

**Por qué**: la condición de LEAVE se basa en criterios de Bug2 formal (cruce de M-line y mejora de distancia), y el arco + refractaria resuelven la persistencia de lecturas frontales altas durante el bordeo.

Definimos la M-line entre el punto de inicio y el objetivo, y evaluamos distancia a la recta, proyección escalar y un umbral angular adaptativo; LEAVE exige además ventana refractaria y ejecuta un arco corto de salida.

$$
d_{\mathcal M}(x,y) = \frac{|v_y x - v_x y + x_2 y_1 - y_2 x_1|}{\sqrt{v_x^2+v_y^2}},\quad
s(x,y) = \frac{(x-x_1)\,v_x + (y-y_1)\,v_y}{\sqrt{v_x^2+v_y^2}}\\
\theta_{allow} = \max\big(\theta_{rec},\ \min(30^\circ,\ 0.2^\circ/\text{cm}\cdot d)\big),\quad d = \sqrt{(x_g-x)^2+(y_g-y)^2}\\
\begin{aligned}
\text{LEAVE si:}\quad & d_{\mathcal M}(x,y) < \varepsilon\ (\approx 3\,\text{cm}) \\
& d < d_{hit} - 2\,\text{cm} \\
& s(x,y) > s_{hit} + 1\,\text{cm} \\
& |e_\theta| \le \theta_{allow} \\
& t - t_{leave} > \tau_{refract}\ (\approx 0.4\,\text{s})
\end{aligned}
$$

Donde:
$$
\begin{aligned}
(x_1,y_1),\ (x_2,y_2) &:\ \text{puntos inicio y objetivo que definen la M-line} \\
(v_x,v_y) &= (x_2-x_1,\,y_2-y_1)\ :\ \text{vector director de la M-line} \\
d_{\mathcal M}(x,y) &:\ \text{distancia punto-recta a la M-line (cm)} \\
s(x,y) &:\ \text{proyección escalar del punto sobre la M-line (cm)} \\
d &:\ \text{distancia actual al objetivo (cm)} \\
\theta_{rec} &:\ \text{umbral base de reacople angular (}\texttt{reacquire\_deg}\text{)} \\
\theta_{allow} &:\ \text{umbral angular adaptativo} \\
s_{hit},\ d_{hit} &:\ \text{proyección y distancia registradas en el impacto} \\
t_{leave},\ \tau_{refract} &:\ \text{tiempo del último LEAVE y ventana refractaria} \\
\varepsilon &:\ \text{tolerancia de cercanía a M-line (≈ 3 cm)}
\end{aligned}
$$

### 6.5 Progreso y recuperación

* Progreso se evalúa por **proyección sobre la M-line** cada `progress_dt_s`.
* Si no hay avance ≥ `progress_eps_cm`, se ejecuta back-off + giro correctivo.

**Por qué**: evita “estáticos” falsos por bordeo y resuelve atascos reales en esquinas concavas.

$$
\Delta s = s(t) - s(t-\Delta t) < \epsilon_{prog} \;\Rightarrow\; \text{back-off + giro},\qquad
\sqrt{(x_g - x)^2 + (y_g - y)^2} \le \text{tol}_{goal} \;\Rightarrow\; \text{éxito}
$$

Donde:
$$
\begin{aligned}
\Delta t &:\ \text{intervalo de evaluación (}\texttt{progress\_dt\_s}\text{)} \\
\epsilon_{prog} &:\ \text{umbral de avance (}\texttt{progress\_eps\_cm}\text{)} \\
\text{tol}_{goal} &:\ \text{tolerancia de llegada (}\texttt{goal\_tolerance\_cm}\text{)}
\end{aligned}
$$

---

## 7. Navegación por Campos de Potencial (PotentialFieldNavigator)

- Integrado en `core/potential_nav.py` usando `core/potential_fields.py`.
- Selección desde GUI: modo "Potential".
- Parámetros en `config.yaml` bajo `potential_nav` (ganancias, límites, umbrales, `default_type`).
- Transformación odometría→mundo basada en `q_i(x,y,theta)` del origen.
- Feedback:
  - LED azul: navegación limpia; naranja/cyan: detección/esquiva activa; verde: éxito.
- Seguridad:
  - Respeta `SafetyMonitorV2`; bumpers provocan recuperación y abortan tras N colisiones.
- Telemetría:
  - `TelemetryLogger.update_command(vl,vr)` en cada iteración.

Flujo rápido de prueba:
- Establecer origen con "Start Nodo" + "Confirmar".
- Elegir "Potential" y "Ir a Nodo" → introducir ID de destino.
- Verificar llegada (pitido) y revisar `nodes/logs/telemetry_*.csv`.

Notas:
- Ganancias iniciales calibradas; ajustar `potential_nav.k_*` si el entorno cambia.
- Tolerancia de llegada en `potential_nav.tolerance_cm` (default 10 cm).

---

## 6.6 Mejoras Implementadas No Documentadas

### 6.6.1 Filtro IIR para Estabilidad de Sensores

**Implementación**: Filtro de paso bajo IIR con α = 0.2 aplicado a todas las lecturas IR.

```python
self._ir_lp = [α * raw[i] + (1.0 - α) * prev[i] for i in range(len(raw))]
```

**Beneficio**: Reduce ruido en lecturas IR sin introducir retraso significativo, mejorando la estabilidad de las decisiones de navegación.

### 6.6.2 Arco de Despegue Post-LEAVE

**Implementación**: Tras cumplir condiciones LEAVE, se ejecuta un arco corto (0.35s) alejándose de la pared.

```python
async def _leave_arc(self, side: str) -> None:
    # Arco hacia el lado opuesto a la pared
    if side == "left":   # pared izquierda -> curva derecha
        vl, vr = base + turn, base - turn
    else:                # pared derecha -> curva izquierda
        vl, vr = base - turn, base + turn
```

**Beneficio**: Reduce lecturas frontales inmediatamente tras LEAVE, evitando reenganche prematuro por persistencia de señales IR altas.

### 6.6.3 Escaneo Inteligente Durante WALL_FOLLOW

**Implementación**: Cuando el frontal se bloquea durante bordeo, usa tres puntos (FL/FC/FR) para encontrar hueco sin detenerse.

```python
def _front_trio(self, ir_vals: List[float]) -> Tuple[float, float, float]:
    fl = ir_vals[self.left_idx[-1]]    # Frontal-izquierda
    fc = max(ir_vals[i] for i in self.front_idx)  # Frontal-central
    fr = ir_vals[self.right_idx[0]]     # Frontal-derecha
    return fl, fc, fr

def _arc_cmd(self, fl: float, fc: float, fr: float) -> Tuple[float, float]:
    diff = (fr - fl)  # Diferencia lateral
    delta = self.ARC_GAIN * diff
    vl = base + delta
    vr = base - delta
```

**Beneficio**: Permite rodear obstáculos complejos sin detenerse completamente, manteniendo fluidez en la navegación.

### 6.6.4 Ventana Refractaria para LEAVE

**Implementación**: Tras ejecutar LEAVE, se aplica una ventana refractaria de 0.4s antes de permitir nuevo LEAVE.

```python
refract_ok = (now - self._last_leave_t) > self._leave_refract_s
```

**Beneficio**: Evita oscilaciones rápidas entre SEEK y WALL_FOLLOW en geometrías complejas.

### 6.6.5 Detección Robusta de Atasco

**Implementación**: Monitorea progreso por proyección escalar en M-line cada 5 segundos.

```python
if STATE == "SEEK" and (now - last_prog_t) > self.PROGRESS_DT:
    if (s_now - s_last) < self.PROGRESS_EPS:
        # Back-off + giro correctivo
        await self._backup_and_turn("right" if heading_err >= 0 else "left")
```

**Beneficio**: Resuelve automáticamente atascos en esquinas cóncavas o geometrías complejas donde el robot puede quedar "atrapado".

### 6.6.6 Selección Robusta de Lado con Margen

**Implementación**: Usa margen del 10% para evitar oscilaciones cuando lecturas laterales son similares.

```python
margin = 0.10 * self.IR_DIR_THRESHOLD
if abs(left_val - right_val) < margin and self._wall_side:
    pass  # mantener lado anterior
else:
    self._wall_side = "left" if left_val < right_val else "right"
```

**Beneficio**: Evita cambios constantes de lado cuando las lecturas IR laterales son muy similares, mejorando estabilidad.

---

## 7. Flujo de uso

### 7.1 Mapeo manual

1. `python teleop_mark_nodes.py`
2. Controles: ↑/←/↓/→ para mover, `G` para guardar nodo, `U` undock, `ESPACIO` freno.
3. Resultado: `nodes.jsonl`, `edges.jsonl`, `logs/edge_*.csv`.

### 7.2 Navegación autónoma

1. `python nav_menu.py`
2. Origen:

   * `Undock` para empezar desde el dock con `reset_navigation()`.
   * `Start Nodo` + `Confirmar` para fijar pose conocida.
3. Destino:

   * `Ir a Nodo` (ID) o `Ir a Nombre`.
   * “Parar” cancela tarea en curso.
4. “Ir a Origen”: va a `(0,0)` con **IR Avoid** (no `navigate_to` directo).
5. Logs: intentos en `nav_attempts.csv`, telemetría continua.

---

## 8. Modos y decisiones de diseño

### 8.1 “Direct IR”

Definimos el modo de navegación con **`IRAvoidNavigator`** porque:

* La consigna es llegar a un punto definido en odometría, **superando cualquier obstáculo sin perder el rumbo**.
* El algoritmo Bug2 con M-line, `hit` y `leave` garantiza reenganche correcto al rumbo objetivo.
* La elección de lado responde a mediciones IR probadas en hardware.

### 8.2 “Replay”

Reproducir aristas con `navigate_to(dx,dy)` puede servir en zonas despejadas o para validar segmentos, pero en presencia de obstáculos se prioriza “Direct IR”.

### 8.3 Origen

Se fija con `Undock` (dock actual) o con un nodo existente, seguido de `reset_navigation()` para establecer un marco relativo consistente.

---

## 9. Seguridad

* `SafetyMonitorV2` se **habilita durante navegación autónoma**.
* Si IR/bumpers/cliff indican peligro: freno momentáneo, LED rojo, bandera `halted`.
* `Override Safety` limpia `halted` cuando el operador lo considera seguro.
* La navegación IR respeta `halted`: no aplica velocidades hasta `clear_halt()`.

Razonamiento: se sincroniza la reacción reactiva con la capa de protección para evitar colisiones.

---

## 10. Telemetría y trazabilidad

* Telemetría periódica a ~10 Hz con los campos clave para diagnóstico causal.
* `nav_attempts.csv`: plan vs resultado, errores de posición/orientación y metadatos del origen.
* CSV de segmentos por arista: permite comparar **planificado vs odometría** por segmento/fase.

Objetivo: auditar decisiones del algoritmo y ajustar parámetros con evidencia.

---

## 11. Visualización y análisis

* `visualize_nodes.py`:

  * Grafo con flechas de orientación y calidad de aristas.
  * Estadísticas globales: distribución de calidad, longitudes, out-degree.
  * Inspección de una arista: errores por segmento, tiempos por estado, comparativas plan/odom.

Uso: `python visualize_nodes.py graph|stats|edge <A> <B>|save <path>`.

---

## 12. Parámetros clave y efectos

* `ir_obs_threshold`:

  * Mayor → más conservador frente a frontal; reduce colisiones pero aumenta longitud de bordeo.
* `ir_dir_threshold`:

  * Mayor → mayor distancia lateral al borde; bordeo más seguro pero más lento.
* `goal_kp`:

  * Mayor → corrige rumbo más rápido; riesgo de sobreviraje.
* `wall_kp`:

  * Mayor → sigue pared más pegado; sensible a ruido IR.
* `reacquire_deg`:

  * Menor → exige alineación estricta para reenganchar M-line; reduce zig-zag.
* `progress_eps_cm`/`progress_dt_s`:

  * Ajustan sensibilidad a "atasco".
* **Histéresis frontal** (T_off = 0.7 × T_on):

  * Factor 0.7 → equilibrio entre estabilidad y responsividad; evita oscilaciones sin ser demasiado conservador.
* **Filtro IIR** (α = 0.2):

  * Mayor α → más responsivo pero más ruido; menor α → más estable pero más lento.
* **Ventana refractaria** (0.4s):

  * Mayor → más estable pero puede ser lento en geometrías complejas; menor → más responsivo pero riesgo de oscilaciones.

---

## 13. Supuestos y limitaciones

* Odometría local estable en distancias de trabajo del laboratorio.
* IR índices y umbrales calibrados para el modelo Create3 usado.
* En geometrías extremadamente concavas o con superficies IR atípicas, puede requerir ajuste fino de umbrales/ganancias.

---

## 14. Pruebas recomendadas

* **Pasillo recto con obstáculo único centrado**: verificar `HIT→WALL_FOLLOW→LEAVE` (sin exigir frontal libre) y regreso estable a M-line.
* **Obstáculo largo (mesa/parede extendida)**: confirmar que el **arco de despegue** reduce el frontal y evita reenganche inmediato; salida respetando ventana refractaria.
* **Esquina cóncava**: comprobar recuperación por back-off + giro cuando no hay progreso escalar en M-line.
* **Cercanía extrema al objetivo sobre la M-line**: validar que el **reacquire adaptativo** permite LEAVE aun con error angular > `reacquire_deg` si la distancia es pequeña.
* **Retorno a origen con objetos interpuestos**: validar que "Ir a Origen" usa IR Avoid y no `navigate_to` directo.
* **Validación de seguridad**: forzar `halted` con cliff/bumpers y confirmar que la navegación no aplica velocidades hasta `clear_halt()`.
* **Obstáculo complejo con múltiples esquinas**: verificar que el **escaneo inteligente** permite rodear sin detenerse completamente.
* **Lecturas IR laterales similares**: confirmar que el **margen del 10%** evita oscilaciones en selección de lado.
* **Ruido en sensores IR**: validar que el **filtro IIR** mantiene estabilidad en decisiones de navegación.
* **Geometría con múltiples obstáculos**: verificar que la **ventana refractaria** evita oscilaciones rápidas entre estados.

---

## 15. Vinculación de archivos y objetivos

* **Definimos el modo de navegación** con `IRAvoidNavigator` en `core/ir_avoid.py` porque, al proyectar el progreso sobre la M-line y usar `HIT/LEAVE`, nos dirigimos a un punto definido mediante odometría **sin perder el rumbo** pese a obstáculos.
* **Persistimos el grafo** con `teleop_mark_nodes.py` + `nodes_io.py` porque necesitamos un conjunto de puntos espaciales verificables y reproducibles.
* **Operamos con seguridad** activando `SafetyMonitorV2` desde `nav_menu.py` para que la reacción ante riesgos de sensores físicos prevalezca sobre cualquier consigna de movimiento.
* **Auditamos el sistema** con `TelemetryLogger` y `log_nav_attempt` para correlacionar decisiones, comandos y resultado espacial.

Esta composición de capas cumple el objetivo académico: mapeo de nodos, navegación inteligente entre ellos con evasión reactiva, y trazabilidad técnica completa.
