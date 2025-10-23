# Comandos del Robot Create 3 - iRobot Python Web Playground

## Eventos (Events)

### Evento de Inicio
```python
@event(robot.when_play)
async def when_play(robot):
    # Los comandos se ejecutarán cuando se presione el botón Play
```

### Sensores de Golpe (Bumpers)
```python
@event(robot.when_bumped, [True, True])
async def bumped(robot):
    # Se activa cuando cualquier sensor de golpe es activado

@event(robot.when_bumped, [True, False])
async def bumped(robot):
    # Se activa cuando el sensor de golpe frontal izquierdo es activado
```
**Descripción:** El robot detectará entrada de sus sensores de golpe izquierdo y/o derecho.

### Sensores Táctiles (Botones)
```python
@event(robot.when_touched, [False, True])
async def touched(robot):
    # Se activa cuando el botón derecho (••) es presionado

@event(robot.when_touched, [True, False])
async def touched(robot):
    # Se activa cuando el botón izquierdo (•) es presionado
```
**Descripción:** En el robot Create 3, hay dos botones. En orden son: izquierdo (•) y derecho (••).

### Sensor de Proximidad IR
```python
@event(robot.when_ir_proximity, threshold)
async def proximity_detected(robot):
    # Se activa cuando se detecta proximidad IR
```

## Comandos de Movimiento

### Movimiento Básico
```python
await robot.move(16)
# Comando para mover el robot hacia adelante una cantidad específica de centímetros
# El ejemplo muestra 16 cm de distancia
```

### Rotación
```python
await robot.turn_left(90)
# Rota el robot en sentido antihorario 90°

await robot.turn_right(90)
# Rota el robot en sentido horario 90°
```
**Descripción:** Comando para que el robot rote en sentido horario (derecha) o antihorario (izquierda) por un número específico de grados.

### Movimiento en Arco
```python
await robot.arc_left(90, 4)
# Robot conduce en arco antihorario 90° con radio de 4 cm

await robot.arc_right(90, 4)
# Robot conduce en arco horario 90° con radio de 4 cm
```
**Descripción:** Comando para que el robot conduzca hacia adelante o atrás en un arco, ya sea en sentido horario (derecha) o antihorario (izquierda) por un número específico de grados a lo largo de una curva de radio específico.

### Sistema de Navegación
```python
await robot.reset_navigation()
# Reinicia el origen de la cuadrícula de navegación del robot a (0, 0, 90°)
# con el centro del robot en el origen cuyo eje X pasa por la rueda derecha
# y el eje Y pasa por el parachoques
```

```python
await robot.navigate_to(16, 16)
# Robot navega a (16 cm, 16 cm) sin establecer orientación final

await robot.navigate_to(0, 0, 45)
# Robot navega a (0 cm, 0 cm) con orientación final de 45 grados
```
**Descripción:** Comando para que el robot navegue a través de una cuadrícula invisible de incrementos de centímetros. La posición del robot se reinicia a (0, 0, 90°) cuando se inicia el programa.

```python
await robot.get_position()
# Comando para que el robot proporcione su ubicación actual basada en 
# una cuadrícula invisible de incrementos de centímetros
```

## Comandos de Audio

### Reproducir Notas
```python
await robot.play_note(440, 0.25)
# Reproduce la nota A4 por 0.25 segundos usando frecuencia

await robot.play_note(Note.A4, 0.25)
# Reproduce la nota A4 por 0.25 segundos usando constante
```
**Descripción:** Comando para que el robot reproduzca una nota musical de una frecuencia específica durante una duración determinada.

```python
await robot.stop_sound()
# El robot detendrá los sonidos que se estén reproduciendo actualmente
```

## Comandos de Luces (LEDs)

### Control de Luces RGB
```python
await robot.set_lights_on_rgb(0, 0, 255)
# Establece las luces del robot en azul, completamente encendidas

await robot.set_lights_spin_rgb(0, 0, 255)
# Establece las luces del robot en azul con animación giratoria

await robot.set_lights_blink_rgb(0, 0, 255)
# Establece las luces del robot en azul con animación parpadeante

await robot.set_lights_off()
# Apaga las luces del robot
```
**Descripción:** Establece las luces del robot para mostrar varios colores y patrones.

## Control de Velocidad de Ruedas

### Velocidad de Ambas Ruedas
```python
await robot.set_wheel_speeds(10, -10)
# Establece la velocidad de la rueda izquierda a 10 cm/s hacia adelante
# y la rueda derecha a 10 cm/s hacia atrás
# El robot rotará en su lugar, en sentido horario
```
**Descripción:** Establece la velocidad de las ruedas del robot (en cm/s) y dirección en cada lado (izquierda/derecha).

### Velocidad Individual de Ruedas
```python
await robot.set_left_speed(5)
# Establece la velocidad del motor de la rueda izquierda

await robot.set_right_speed(-5)
# Establece la velocidad del motor de la rueda derecha
```
**Descripción:** Establece la velocidad de uno de los motores de rueda del robot (en cm/s); el otro continuará sin cambios.

## Configuración del Robot

### Nombre del Robot
```python
await robot.set_name('Flea')
# Establece el nombre del robot como "Flea"
```

## Control de Flujo

### Esperas y Pausas
```python
await robot.wait(0.5)
# Especifica una demora de 0.5 segundos antes de que el robot
# pase a la siguiente línea de código
```

```python
await hand_over()
# Python parece ejecutar todas las tareas (@events) de forma concurrente
# Las tareas no se ejecutan realmente al mismo tiempo, sino que comparten
# tiempo en el procesador. Es importante llamar await hand_over() para
# indicar al programa que está bien ceder tiempo a otras tareas
```

## Comandos de Información (Getters)

### Información del Sistema
```python
await robot.get_version_string()
# Obtiene la versión del robot como una cadena legible

await robot.get_name()
# Obtiene el nombre del robot

await robot.get_serial_number()
# Obtiene el número de serie del robot

await robot.get_battery_level()
# Obtiene el nivel de batería del robot

await robot.get_ipv4_address()
# Obtiene las direcciones IP del robot para todas sus interfaces (Solo Create 3)
```

### Sensores
```python
await robot.get_position()
# Obtiene la posición interna actual del robot

await robot.get_accelerometer()
# Obtiene los valores actuales reportados por el acelerómetro del robot

await robot.get_bumpers()
# Obtiene el estado actual de los sensores de golpe

await robot.get_touch_sensors()
# Obtiene los valores actuales reportados por los sensores táctiles del robot

await robot.get_ir_proximity()
# Obtiene los valores de los sensores de proximidad IR del robot (Solo Create 3)
```

## Referencias de Colores RGB

### Colores al 100% de Brillo
- **Rojo:** `(255, 0, 0)`
- **Naranja:** `(255, 64, 0)`
- **Amarillo:** `(255, 115, 0)`
- **Verde:** `(0, 255, 0)`
- **Cian:** `(101, 197, 181)`
- **Azul:** `(0, 0, 255)`
- **Púrpura:** `(111, 71, 127)`
- **Blanco:** `(255, 255, 255)`
- **Gris:** `(60, 60, 60)`

### Colores al 50% de Brillo
- **Rojo:** `(127, 0, 0)`
- **Naranja:** `(127, 32, 0)`
- **Amarillo:** `(127, 57, 0)`
- **Verde:** `(0, 127, 0)`
- **Cian:** `(50, 98, 90)`
- **Azul:** `(0, 0, 127)`
- **Púrpura:** `(55, 35, 63)`
- **Gris claro:** `(127, 127, 127)`
- **Negro:** `(0, 0, 0)`

## Notas Musicales

### Ejemplos de Constantes de Notas
- **A4 (La):** `Note.A4` o `440` Hz
- **C4 (Do):** `Note.C4` o `261` Hz
- **E4 (Mi):** `Note.E4` o `329` Hz
- **G4 (Sol):** `Note.G4` o `391` Hz

### Rango de Notas Disponibles
El robot admite notas desde A0 (27.50 Hz) hasta C8 (4186 Hz), con todas las notas intermedias incluyendo sostenidos y bemoles.

## Ejemplo de Programa Completo

```python
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Note, Robot

robot = Robot(Bluetooth())

@event(robot.when_play)
async def when_play(robot):
    # Configurar luces azules
    await robot.set_lights_on_rgb(0, 0, 255)
    
    # Mover hacia adelante
    await robot.move(30)
    
    # Rotar 90 grados
    await robot.turn_right(90)
    
    # Reproducir una nota
    await robot.play_note(Note.A4, 0.5)
    
    # Apagar luces
    await robot.set_lights_off()
```

---

**Nota:** Este documento está basado en la versión 2022 de iRobot Education para uso con el robot Create 3 en el iRobot Python Web Playground.