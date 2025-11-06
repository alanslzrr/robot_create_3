#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Configurador Visual de Puntos de Navegación para Robot Create 3

Autores: Alan Salazar, Yago Ramos
Fecha: 6 de noviembre de 2025
Institución: UIE Universidad Intercontinental de la Empresa
Asignatura: Robots Autónomos - Profesor Eladio Dapena

DESCRIPCIÓN:

Este script proporciona una interfaz gráfica interactiva para configurar los puntos
de navegación del robot de manera intuitiva y visual. El usuario puede hacer clic
en un mapa para establecer las posiciones inicial y final, y además definir la
orientación inicial del robot mediante una flecha visual.

CARACTERÍSTICAS PRINCIPALES:

1. **Mapa Interactivo de 500x500 cm**: Representa el espacio de navegación del robot
   con una cuadrícula visual que facilita la colocación precisa de puntos.

2. **Configuración Visual de Orientación**: El usuario puede definir la orientación
   inicial del robot mediante una flecha que indica hacia dónde apunta el robot.
   La orientación se calcula automáticamente o se puede ajustar manualmente.

3. **Validación en Tiempo Real**: El sistema valida que los puntos estén separados
   adecuadamente y muestra advertencias si están demasiado cerca.

4. **Generación Automática de JSON**: Guarda la configuración en formato JSON
   compatible con los scripts principales (PRM01_P01.py y PRM01_P02.py).

5. **Vista Previa del Robot**: Muestra una representación visual del robot en la
   posición inicial con su orientación, ayudando a visualizar el setup completo.

MODO DE USO:

1. Ejecutar el script:
   python utils/visual_point_config.py

2. Hacer clic en el mapa para colocar el PUNTO INICIAL (q_i) - marcado en verde

3. Hacer clic nuevamente para definir la DIRECCIÓN INICIAL del robot
   (esto establece hacia dónde apunta el robot en el punto inicial)

4. Hacer clic para colocar el PUNTO FINAL (q_f) - marcado en rojo

5. Los puntos se pueden reconfigurar:
   - Presionar 'R' para resetear y comenzar de nuevo
   - Presionar 'S' para guardar la configuración actual
   - Presionar 'Q' para salir sin guardar

6. El archivo JSON se guarda automáticamente en data/points.json

SISTEMA DE COORDENADAS:

- Origen (0,0) en la esquina inferior izquierda
- Eje X horizontal: de 0 a 500 cm (de izquierda a derecha)
- Eje Y vertical: de 0 a 500 cm (de abajo hacia arriba)
- Ángulo theta: en grados, medido desde el eje X positivo
  - 0° apunta hacia la derecha (+X)
  - 90° apunta hacia arriba (+Y)
  - 180° o -180° apunta hacia la izquierda (-X)
  - -90° apunta hacia abajo (-Y)

FORMATO DEL JSON GENERADO:

{
    "q_i": {
        "x": <coordenada_x_inicial>,
        "y": <coordenada_y_inicial>,
        "theta": <orientacion_en_grados>
    },
    "q_f": {
        "x": <coordenada_x_final>,
        "y": <coordenada_y_final>
    }
}

NOTAS IMPORTANTES:

- El theta (orientación) es CRÍTICO para el funcionamiento correcto del robot
- Una orientación mal configurada causa que el robot gire innecesariamente
- La orientación debe apuntar aproximadamente hacia el objetivo para navegación eficiente
- El sistema valida que los puntos estén al menos a 10 cm de distancia
"""

import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrow, Circle
import numpy as np


class VisualPointConfigurator:
    """
    Configurador visual interactivo de puntos de navegación.
    
    Esta clase maneja toda la lógica de la interfaz gráfica, incluyendo:
    - Renderizado del mapa de navegación
    - Captura de eventos de mouse y teclado
    - Validación de puntos configurados
    - Generación del archivo JSON
    """
    
    def __init__(self, map_size=500):
        """
        Inicializa el configurador visual.
        
        Args:
            map_size: Tamaño del mapa cuadrado en centímetros (default: 500)
        """
        self.map_size = map_size
        self.output_file = Path("data/points.json")
        
        # Estado de configuración
        self.q_i = None  # Punto inicial (x, y)
        self.orientation_point = None  # Punto para definir orientación
        self.q_f = None  # Punto final (x, y)
        self.theta = None  # Orientación en grados
        
        # Control del flujo de configuración
        self.step = 0  # 0: esperando q_i, 1: esperando orientación, 2: esperando q_f, 3: completado
        
        # Elementos visuales
        self.fig = None
        self.ax = None
        self.point_initial = None
        self.point_final = None
        self.orientation_arrow = None
        self.robot_circle = None
        self.info_text = None
        self.orientation_label = None
        
        # Control de arrastre para orientación
        self.dragging_orientation = False
        self.temp_orientation_arrow = None
        
        # Configurar la interfaz gráfica
        self._setup_plot()
        
    def _setup_plot(self):
        """Configura la figura de matplotlib y el mapa interactivo."""
        # Crear figura con tamaño adecuado
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('Configurador Visual de Puntos - Robot Create 3')
        
        # Configurar el área del mapa con margen extra para permitir orientación en todos los ángulos
        # Extendemos los límites para permitir clicks fuera del área principal
        margin = 100  # 100 cm de margen extra en cada lado
        self.ax.set_xlim(-margin, self.map_size + margin)
        self.ax.set_ylim(-margin, self.map_size + margin)
        self.ax.set_aspect('equal')
        
        # Etiquetas y título
        self.ax.set_xlabel('X (cm)', fontsize=12, fontweight='bold')
        self.ax.set_ylabel('Y (cm)', fontsize=12, fontweight='bold')
        self.ax.set_title('Configuración de Puntos de Navegación\n' +
                         'Paso 1: Click para PUNTO INICIAL (verde)',
                         fontsize=14, fontweight='bold', pad=20)
        
        # Agregar cuadrícula para facilitar colocación de puntos
        self.ax.grid(True, linestyle='--', alpha=0.4, linewidth=0.8)
        self.ax.set_axisbelow(True)
        
        # Agregar cuadrícula mayor cada 50 cm (solo en el área del mapa)
        major_ticks_x = np.arange(0, self.map_size + 1, 50)
        major_ticks_y = np.arange(0, self.map_size + 1, 50)
        self.ax.set_xticks(major_ticks_x)
        self.ax.set_yticks(major_ticks_y)
        
        # Cuadrícula menor cada 10 cm (solo en el área del mapa)
        minor_ticks_x = np.arange(0, self.map_size + 1, 10)
        minor_ticks_y = np.arange(0, self.map_size + 1, 10)
        self.ax.set_xticks(minor_ticks_x, minor=True)
        self.ax.set_yticks(minor_ticks_y, minor=True)
        self.ax.grid(which='minor', linestyle=':', alpha=0.2, linewidth=0.5)
        
        # Sombrear el área fuera del mapa para indicar zona de orientación
        from matplotlib.patches import Rectangle
        # Área gris arriba
        self.ax.add_patch(Rectangle((0, self.map_size), self.map_size, margin, 
                                   facecolor='lightgray', alpha=0.15, zorder=0))
        # Área gris a la derecha
        self.ax.add_patch(Rectangle((self.map_size, 0), margin, self.map_size, 
                                   facecolor='lightgray', alpha=0.15, zorder=0))
        # Área gris abajo
        self.ax.add_patch(Rectangle((0, -margin), self.map_size, margin, 
                                   facecolor='lightgray', alpha=0.15, zorder=0))
        # Área gris a la izquierda
        self.ax.add_patch(Rectangle((-margin, 0), margin, self.map_size, 
                                   facecolor='lightgray', alpha=0.15, zorder=0))
        # Esquinas
        self.ax.add_patch(Rectangle((-margin, -margin), margin, margin, 
                                   facecolor='lightgray', alpha=0.15, zorder=0))
        self.ax.add_patch(Rectangle((self.map_size, -margin), margin, margin, 
                                   facecolor='lightgray', alpha=0.15, zorder=0))
        self.ax.add_patch(Rectangle((-margin, self.map_size), margin, margin, 
                                   facecolor='lightgray', alpha=0.15, zorder=0))
        self.ax.add_patch(Rectangle((self.map_size, self.map_size), margin, margin, 
                                   facecolor='lightgray', alpha=0.15, zorder=0))
        
        # Agregar borde al mapa
        border = patches.Rectangle((0, 0), self.map_size, self.map_size,
                                   linewidth=3, edgecolor='black',
                                   facecolor='none')
        self.ax.add_patch(border)
        
        # Agregar texto de instrucciones en la parte inferior
        instructions = (
            "CONTROLES:\n"
            "  Click Izquierdo: Colocar puntos\n"
            "  Tecla 'R': Resetear configuración\n"
            "  Tecla 'S': Guardar y salir\n"
            "  Tecla 'Q': Salir sin guardar"
        )
        self.fig.text(0.02, 0.02, instructions, fontsize=10,
                     verticalalignment='bottom',
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Conectar eventos
        self.fig.canvas.mpl_connect('button_press_event', self._on_mouse_press)
        self.fig.canvas.mpl_connect('button_release_event', self._on_mouse_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_mouse_move)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        
    def _on_mouse_press(self, event):
        """
        Maneja eventos de presión del mouse.
        
        Args:
            event: Evento de matplotlib con información del click
        """
        # Para la orientación, permitir clicks en toda el área extendida
        if self.step == 1:
            # En paso de orientación, permitir cualquier posición (incluso fuera del mapa)
            if event.xdata is not None and event.ydata is not None:
                x, y = event.xdata, event.ydata
                self.dragging_orientation = True
                self._update_orientation_preview(x, y)
            return
        
        # Para otros pasos, verificar que tengamos coordenadas válidas
        if event.xdata is None or event.ydata is None:
            return
        
        x, y = event.xdata, event.ydata
        
        # Validar que los puntos inicial y final estén dentro del mapa
        if x < 0 or x > self.map_size or y < 0 or y > self.map_size:
            print(f"\n[ADVERTENCIA] Los puntos deben estar dentro del mapa (0-{self.map_size} cm)")
            return
        
        if self.step == 0:
            # Paso 1: Definir punto inicial
            self._set_initial_point(x, y)
            
        elif self.step == 2:
            # Paso 3: Definir punto final
            self._set_final_point(x, y)
    
    def _on_mouse_release(self, event):
        """
        Maneja eventos de liberación del mouse.
        
        Args:
            event: Evento de matplotlib con información del release
        """
        if self.step == 1 and self.dragging_orientation:
            # Finalizar la configuración de orientación
            if event.xdata is not None and event.ydata is not None:
                x, y = event.xdata, event.ydata
                self._set_orientation(x, y)
            self.dragging_orientation = False
    
    def _on_mouse_move(self, event):
        """
        Maneja eventos de movimiento del mouse.
        
        Args:
            event: Evento de matplotlib con información del movimiento
        """
        if self.step == 1 and self.dragging_orientation:
            # Actualizar vista previa de orientación mientras arrastra
            if event.xdata is not None and event.ydata is not None:
                x, y = event.xdata, event.ydata
                self._update_orientation_preview(x, y)
    
    def _update_orientation_preview(self, x, y):
        """
        Actualiza la vista previa de la orientación mientras el usuario arrastra.
        
        Args:
            x, y: Coordenadas actuales del mouse
        """
        if self.q_i is None:
            return
        
        # Calcular ángulo desde punto inicial hacia posición del mouse
        dx = x - self.q_i[0]
        dy = y - self.q_i[1]
        
        # Validar distancia mínima para evitar ángulos inestables
        distance = math.hypot(dx, dy)
        if distance < 5:
            return
        
        # Calcular theta temporal
        temp_theta = math.degrees(math.atan2(dy, dx))
        
        # Limpiar flecha temporal anterior
        if self.temp_orientation_arrow:
            self.temp_orientation_arrow.remove()
            self.temp_orientation_arrow = None
        
        if self.orientation_label:
            self.orientation_label.remove()
            self.orientation_label = None
        
        # Dibujar flecha temporal
        arrow_length = 40
        arrow_dx = arrow_length * math.cos(math.radians(temp_theta))
        arrow_dy = arrow_length * math.sin(math.radians(temp_theta))
        
        self.temp_orientation_arrow = FancyArrow(
            self.q_i[0], self.q_i[1],
            arrow_dx, arrow_dy,
            width=5, head_width=15, head_length=10,
            color='blue', alpha=0.5, zorder=6
        )
        self.ax.add_patch(self.temp_orientation_arrow)
        
        # Agregar etiqueta temporal
        label_x = self.q_i[0] + arrow_dx * 0.6
        label_y = self.q_i[1] + arrow_dy * 0.6
        self.orientation_label = self.ax.text(
            label_x, label_y, f'θ = {temp_theta:.1f}°',
            ha='center', va='center', fontweight='bold',
            color='darkblue', fontsize=10,
            bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.5)
        )
        
        self.fig.canvas.draw_idle()
    
    def _set_initial_point(self, x, y):
        """Establece el punto inicial del robot."""
        self.q_i = (round(x, 2), round(y, 2))
        
        # Dibujar punto inicial en verde
        if self.point_initial:
            self.point_initial.remove()
        
        self.point_initial = Circle((x, y), radius=8, color='green',
                                   alpha=0.7, zorder=5)
        self.ax.add_patch(self.point_initial)
        
        # Agregar etiqueta
        self.ax.text(x, y + 15, f'INICIO\n({x:.1f}, {y:.1f})',
                    ha='center', va='bottom', fontweight='bold',
                    color='darkgreen', fontsize=10,
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgreen', alpha=0.7))
        
        # Actualizar título
        self.ax.set_title('Configuración de Puntos de Navegación\n' +
                         'Paso 2: ARRASTRA para definir ORIENTACIÓN (mantén click y mueve el mouse)',
                         fontsize=14, fontweight='bold', pad=20)
        
        self.step = 1
        self.fig.canvas.draw()
    
    def _set_orientation(self, x, y):
        """Establece la orientación inicial del robot."""
        self.orientation_point = (round(x, 2), round(y, 2))
        
        # Calcular ángulo desde punto inicial hacia punto de orientación
        dx = x - self.q_i[0]
        dy = y - self.q_i[1]
        
        # Validar que el punto de orientación no esté demasiado cerca
        distance = math.hypot(dx, dy)
        if distance < 5:
            print("\n[ADVERTENCIA] El punto de orientación está muy cerca del punto inicial.")
            print("              Arrastra más lejos para definir mejor la dirección.")
            return
        
        # Calcular theta en grados
        self.theta = round(math.degrees(math.atan2(dy, dx)), 2)
        
        # Limpiar flecha temporal si existe
        if self.temp_orientation_arrow:
            self.temp_orientation_arrow.remove()
            self.temp_orientation_arrow = None
        
        if self.orientation_label:
            self.orientation_label.remove()
            self.orientation_label = None
        
        # Dibujar flecha de orientación FINAL
        if self.orientation_arrow:
            self.orientation_arrow.remove()
        
        # Normalizar la longitud de la flecha a 40 cm
        arrow_length = 40
        arrow_dx = arrow_length * math.cos(math.radians(self.theta))
        arrow_dy = arrow_length * math.sin(math.radians(self.theta))
        
        self.orientation_arrow = FancyArrow(
            self.q_i[0], self.q_i[1],
            arrow_dx, arrow_dy,
            width=5, head_width=15, head_length=10,
            color='blue', alpha=0.8, zorder=6
        )
        self.ax.add_patch(self.orientation_arrow)
        
        # Dibujar círculo que representa al robot
        if self.robot_circle:
            self.robot_circle.remove()
        
        # Radio del robot Create 3 es aproximadamente 16 cm
        self.robot_circle = Circle((self.q_i[0], self.q_i[1]), radius=16,
                                  color='blue', alpha=0.2, zorder=4,
                                  linestyle='--', linewidth=2, fill=True)
        self.ax.add_patch(self.robot_circle)
        
        # Agregar etiqueta de orientación FINAL
        label_x = self.q_i[0] + arrow_dx * 0.6
        label_y = self.q_i[1] + arrow_dy * 0.6
        final_label = self.ax.text(label_x, label_y, f'θ = {self.theta:.1f}°',
                    ha='center', va='center', fontweight='bold',
                    color='darkblue', fontsize=10,
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.8))
        
        # Actualizar título
        self.ax.set_title('Configuración de Puntos de Navegación\n' +
                         'Paso 3: Click para PUNTO FINAL (rojo)',
                         fontsize=14, fontweight='bold', pad=20)
        
        self.step = 2
        self.fig.canvas.draw()
    
    def _set_final_point(self, x, y):
        """Establece el punto final del robot."""
        self.q_f = (round(x, 2), round(y, 2))
        
        # Validar distancia entre puntos
        distance = math.hypot(self.q_f[0] - self.q_i[0],
                             self.q_f[1] - self.q_i[1])
        
        if distance < 10:
            print("\n[ADVERTENCIA] El punto final está muy cerca del punto inicial.")
            print(f"              Distancia: {distance:.1f} cm (mínimo recomendado: 10 cm)")
            print("              Considera colocar un punto final más alejado.")
            # No avanzar de paso, permitir reconfigurar
            return
        
        # Dibujar punto final en rojo
        if self.point_final:
            self.point_final.remove()
        
        self.point_final = Circle((x, y), radius=8, color='red',
                                 alpha=0.7, zorder=5)
        self.ax.add_patch(self.point_final)
        
        # Agregar etiqueta
        self.ax.text(x, y - 15, f'META\n({x:.1f}, {y:.1f})',
                    ha='center', va='top', fontweight='bold',
                    color='darkred', fontsize=10,
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='lightcoral', alpha=0.7))
        
        # Dibujar línea punteada entre inicio y fin
        self.ax.plot([self.q_i[0], self.q_f[0]],
                    [self.q_i[1], self.q_f[1]],
                    'k--', linewidth=2, alpha=0.4, zorder=3)
        
        # Agregar información de distancia
        mid_x = (self.q_i[0] + self.q_f[0]) / 2
        mid_y = (self.q_i[1] + self.q_f[1]) / 2
        self.ax.text(mid_x, mid_y, f'Distancia: {distance:.1f} cm',
                    ha='center', va='center', fontweight='bold',
                    color='black', fontsize=9,
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7))
        
        # Actualizar título
        self.ax.set_title('Configuración Completada\n' +
                         'Presiona "S" para GUARDAR o "R" para RESETEAR',
                         fontsize=14, fontweight='bold', pad=20,
                         color='darkgreen')
        
        self.step = 3
        self.fig.canvas.draw()
        
        # Mostrar resumen en consola
        self._print_summary()
    
    def _on_key(self, event):
        """
        Maneja eventos de teclado.
        
        Args:
            event: Evento de matplotlib con información de la tecla presionada
        """
        if event.key.lower() == 'r':
            # Resetear configuración
            self._reset()
            
        elif event.key.lower() == 's':
            # Guardar configuración
            if self.step == 3:
                self._save_configuration()
                plt.close(self.fig)
            else:
                print("\n[ERROR] Configuración incompleta. Completa todos los pasos antes de guardar.")
                
        elif event.key.lower() == 'q':
            # Salir sin guardar
            print("\n[INFO] Saliendo sin guardar...")
            plt.close(self.fig)
    
    def _reset(self):
        """Resetea la configuración y limpia el mapa."""
        print("\n[INFO] Reseteando configuración...")
        
        # Limpiar estado
        self.q_i = None
        self.orientation_point = None
        self.q_f = None
        self.theta = None
        self.step = 0
        
        # Limpiar elementos visuales
        if self.point_initial:
            self.point_initial.remove()
            self.point_initial = None
        
        if self.point_final:
            self.point_final.remove()
            self.point_final = None
        
        if self.orientation_arrow:
            self.orientation_arrow.remove()
            self.orientation_arrow = None
        
        if self.robot_circle:
            self.robot_circle.remove()
            self.robot_circle = None
        
        if self.temp_orientation_arrow:
            self.temp_orientation_arrow.remove()
            self.temp_orientation_arrow = None
        
        if self.orientation_label:
            self.orientation_label.remove()
            self.orientation_label = None
        
        # Limpiar textos adicionales
        for txt in self.ax.texts[:]:
            txt.remove()
        
        # Limpiar líneas adicionales
        for line in self.ax.lines[:]:
            line.remove()
        
        # Resetear estado de arrastre
        self.dragging_orientation = False
        
        # Resetear título
        self.ax.set_title('Configuración de Puntos de Navegación\n' +
                         'Paso 1: Click para PUNTO INICIAL (verde)',
                         fontsize=14, fontweight='bold', pad=20)
        
        self.fig.canvas.draw()
    
    def _print_summary(self):
        """Imprime un resumen de la configuración actual."""
        print("\n" + "="*60)
        print("CONFIGURACIÓN DE PUNTOS COMPLETADA")
        print("="*60)
        print(f"\nPunto Inicial (q_i):")
        print(f"   x = {self.q_i[0]:.2f} cm")
        print(f"   y = {self.q_i[1]:.2f} cm")
        print(f"   theta = {self.theta:.2f} deg")
        print(f"\nPunto Final (q_f):")
        print(f"   x = {self.q_f[0]:.2f} cm")
        print(f"   y = {self.q_f[1]:.2f} cm")
        
        distance = math.hypot(self.q_f[0] - self.q_i[0],
                             self.q_f[1] - self.q_i[1])
        angle_to_goal = math.degrees(math.atan2(
            self.q_f[1] - self.q_i[1],
            self.q_f[0] - self.q_i[0]
        ))
        
        print(f"\nDistancia a recorrer: {distance:.1f} cm")
        print(f"Ángulo hacia meta: {angle_to_goal:.1f} deg")
        
        # Calcular error angular inicial
        angle_diff = angle_to_goal - self.theta
        # Normalizar a rango [-180, 180]
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff <= -180:
            angle_diff += 360
        
        print(f"Error angular inicial: {angle_diff:.1f} deg")
        
        if abs(angle_diff) > 45:
            print("\n[ADVERTENCIA] El robot tendrá que girar significativamente al inicio.")
            print("              Considera ajustar la orientación inicial para mejor eficiencia.")
        
        print("="*60)
        print("\nPresiona 'S' para GUARDAR o 'R' para RESETEAR y reconfigurar.")
    
    def _save_configuration(self):
        """Guarda la configuración actual en el archivo JSON."""
        # Crear estructura de datos
        data = {
            "q_i": {
                "x": self.q_i[0],
                "y": self.q_i[1],
                "theta": self.theta
            },
            "q_f": {
                "x": self.q_f[0],
                "y": self.q_f[1]
            }
        }
        
        # Asegurar que el directorio existe
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        
        # Guardar JSON con formato bonito
        try:
            with open(self.output_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4, ensure_ascii=False)
            
            print(f"\n[ÉXITO] Configuración guardada en: {self.output_file}")
            print("\nPuedes usar esta configuración ejecutando:")
            print("  python PRM01_P01.py --potential linear")
            print("  python PRM01_P02.py --potential conic")
            
        except Exception as e:
            print(f"\n[ERROR] No se pudo guardar el archivo: {e}")
            sys.exit(1)
    
    def run(self):
        """Ejecuta la interfaz gráfica interactiva."""
        print("\n" + "="*60)
        print("CONFIGURADOR VISUAL DE PUNTOS DE NAVEGACIÓN")
        print("="*60)
        print("\nInstrucciones:")
        print("1. Haz click DENTRO del mapa para colocar el PUNTO INICIAL (verde)")
        print("2. ARRASTRA (mantén click y mueve) para definir la ORIENTACIÓN")
        print("   - Puedes arrastrar FUERA del mapa (área gris) para ángulos negativos")
        print("   - La flecha indica hacia dónde apunta el robot")
        print("   - Funciona en todas las direcciones: 0°, 90°, 180°, -90°, etc.")
        print("3. Haz click DENTRO del mapa para colocar el PUNTO FINAL (rojo)")
        print("4. Presiona 'S' para guardar o 'R' para resetear")
        print("\nSistema de coordenadas:")
        print("  - Origen (0,0) en esquina inferior izquierda")
        print("  - Theta 0° = derecha (+X), 90° = arriba (+Y)")
        print("  - Theta 180° = izquierda (-X), -90° = abajo (-Y)")
        print("="*60 + "\n")
        
        plt.show()


def main():
    """Función principal del script."""
    # Verificar que matplotlib esté instalado
    try:
        import matplotlib
    except ImportError:
        print("\n[ERROR] Este script requiere matplotlib.")
        print("        Instala con: pip install matplotlib")
        sys.exit(1)
    
    # Crear y ejecutar configurador
    configurator = VisualPointConfigurator(map_size=500)
    configurator.run()


if __name__ == "__main__":
    main()
