
# 🧠 Plantilla de Contexto para Proyecto de Robot (Formato para uso con GPT personalizado)

Esta plantilla está diseñada para documentar de forma detallada la información de tu robot y su proyecto asociado. Se usará como fuente de conocimiento para un GPT personalizado que asistirá a estudiantes. Es fundamental ser preciso y completo para obtener las mejores respuestas posibles.

---

## 1. 📘 Descripción General del Proyecto

- **Nombre del Proyecto**:
- **Curso / Institución**:
- **Semestre / Año**:
- **Nombre del Profesor**:
- **Público Objetivo** (ej. estudiantes de último semestre en robótica):
- **Objetivos de Aprendizaje** (mínimo 3):
  - 
  - 
  - 
- **Resumen del Proyecto**: Describe brevemente en qué consiste el proyecto.
- **Fases del Proyecto** (ej. simulación → implementación en robot real):

---

## 2. 🤖 Especificaciones del Robot

### 2.1 En Simulación
- **Nombre del Robot**:
- **Tipo de plataforma**: (ej. tracción diferencial, omnidireccional). Describe cómo se mueve el robot.
- **Dimensiones (simuladas)**: tamaño del chasis, posición relativa de sensores, etc.
- **Plugins usados**: ¿Se usó el plugin de Gazebo para dif_drive? ¿Se implementó un nodo propio?
- **Sensores Simulados**:
  - LiDAR: modelo, rango, FOV, frame_id, topic
  - Cámara: modelo, resolución, topic, calibración usada (si aplica)
  - Otros sensores: TOF, IMU, etc.
- **Archivos URDF/XACRO**: ¿Dónde están definidos los modelos? ¿Qué nombres de frame usan?
- **Entorno simulado**: ¿Se diseñó un laberinto en Blender? ¿Cómo se exportó?

### 2.2 En Robot Físico
- **Nombre del Robot**:
- **Fabricante**:
- **Tipo de plataforma**: igual que en simulación. Explica diferencias si las hay.
- **Radio de Rueda (en metros)**:
- **Separación entre Ruedas (en metros)**:
- **Tipo de tracción**: (diferencial, etc.)
- **Sensores Físicos**:
  - LiDAR físico: modelo, puerto de conexión, frame_id
  - Cámara física: tipo de interfaz (CSI, USB), resolución, nodo ROS usado
  - Otros sensores físicos: TOF, encoder de ruedas, IMU
- **Controladores**:
  - Microcontrolador: modelo (ej. ESP32)
  - Computadora principal: (ej. Jetson Nano, specs, fuente de alimentación)
- **Sistema Operativo**:
  - OS del Jetson:
  - ROS 2 distro usada:
  - RMW y configuración en `.bashrc`:

---

## 3. 🧰 Stack de Software

- **Distribución de ROS 2**:
- **Entorno de Simulación**:
- **Frameworks / Stacks usados**:
  - Nav2
  - micro-ROS
  - SLAM Toolbox
  - AMCL
  - ¿Otros?
- **Nodos personalizados creados**:
  - Nombre, funcionalidad y cómo se integran
- **Nodos clave que se ejecutan** (en Jetson o PC):
  - ¿Cómo se lanza el agente micro-ROS?
  - ¿Qué tópicos produce y consume cada nodo?

---

## 4. 🗂 Archivos de Configuración

### 4.1 Contenido Relevante en `.bashrc`
Incluye todas las variables que se modificaron. Por ejemplo:
```
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
```

### 4.2 Árbol de Transformaciones TF
- Describe el árbol esperado (`map → odom → base_link → laser`)
- Anota transformaciones estáticas necesarias
- Explica errores comunes que encontraste y cómo los resolviste

---

## 5. 🔄 Integración con Proyecto de Manchester

- ¿Qué publicaban los nodos originales de Manchester? ¿Tenían prefijos?
  - `/odom`, `/tf`, `/cmd_vel`, etc.
- ¿Qué tópicos crearon tus propios nodos? ¿Con qué tipo de mensajes?
- ¿Qué eliminaste de Gazebo para no duplicar funcionalidades?
- ¿Cómo aseguraste consistencia entre simulación y robot real?

---

## 6. ❗ Retos Frecuentes por Categoría

### 6.1 En Simulación
Describe problemas que enfrentaste en simulación:
- ¿Cómo se manifestaban? (ej. "el robot se teletransporta en RViz")
- ¿Qué mensajes de consola aparecían?
- ¿Cómo los solucionaste paso a paso?

### 6.2 En Robot Físico
Problemas reales que enfrentaste:
- ¿Cómo te diste cuenta del error?
- ¿Qué pasos seguiste para depurar y resolver?

---

## 7. 🚧 Malentendidos Comunes

Lista de conceptos que tú (o tu equipo) asumieron erróneamente y que resultaron ser diferentes:
- Ej. "Creí que `use_sim_time:=true` era suficiente, pero no lo configuré en los `.yaml`"
- Explica cómo esos errores afectaron el desempeño del proyecto

---

## 8. 📸 Recursos Visuales

Lista de imágenes que acompañan tu archivo:
- `hardware_puzzlebot.jpg`: vista general del robot
- `rqt_graph.png`: muestra de nodos y tópicos
- `rviz_config.png`: vista de navegación
- `tf_tree_example.png`: transformaciones TF
- `maze_blender_export.png`: entorno simulado

---

## 9. 🪜 Pasos del Proyecto (Timeline)

Lista ordenada de pasos que seguiste desde la simulación hasta la ejecución real:

1. Crear repositorio en GitHub
2. Integrar el proyecto base de navegación de Turtlebot3 al robot Puzzlebot
3. Diseñar el entorno (laberinto) usando Blender
4. Implementar el stack de navegación en simulación
5. Afinar parámetros de Nav2 (planner, controller)
6. Configurar robot físico (pasos detallados: micro-ROS, URDF, etc.)
7. Ajustar TFs, tópicos y controladores reales
8. Validar movimiento autónomo en entorno físico

---

## 10. 🔗 Enlaces Útiles

Incluye todas las páginas, videos, foros, tutoriales que consultaste:
- [Tutorial de micro-ROS](https://...)
- [Foro de Nav2 Issues](https://...)
- [Documentación oficial de AMCL](https://...)

## Incluir cualquier otra sección que en su experiencia consideren relevante. Algo que les hubiera gustado haber sabido en un inicio y que les hubiera hecho el desarrollo del proyecto más sencillo. 