# 🧠 Plantilla de Contexto para Proyecto de Robot (Formato para uso con GPT personalizado)

Esta plantilla está diseñada para documentar de forma detallada la información de tu robot y su proyecto asociado. Se usará como fuente de conocimiento para un GPT personalizado que asistirá a estudiantes. Es fundamental ser preciso y completo para obtener las mejores respuestas posibles.

---

## 1. 📘 Descripción General del Proyecto

- **Nombre del Proyecto**: Desarrollo e Implementación de Algoritmos de Localización y Navegación Autónoma a través de LiDAR para Robot Móvil. 
- **Curso / Institución**: Integración de Robótica y Sistemas Inteligentes / Tecnológico de Monterrey
- **Semestre / Año**: Feb-Jun 2025
- **Nombre del Profesor**: Juan Manuel Ledesma Rangel
- **Público Objetivo** Estudiantes de último semestre en robótica y personas interesadas en la navegación autónoma:
- **Objetivos de Aprendizaje** (mínimo 3):
  - Utilizar SLAM Toolbox para generar y refinar mapas del entorno en tiempo real.
  - Organizar y gestionar múltiples archivos de parámetros para modular la configuración del stack
  - Configurar y ejecutar el stack de navegación Nav2 en un entorno ROS 2 real y simulado.
  - Diagnosticar y solucionar problemas comunes al implementar navegación con Nav2 y SLAM
  - Diseñar algoritmos de navegación autónoma. 
Toolbox.
- **Resumen del Proyecto**: Implementación de un sistema de navegación autónoma usando el stack Nav2 y SLAM Toolbox sobre un robot diferencial Puzzlebot. El proyecto permite que el robot mapee su entorno en tiempo real y navegue de forma segura hasta metas definidas, evitando obstáculos mediante el uso de LiDAR, costmaps y planificación de trayectoria en ROS 
- **Fases del Proyecto** (ej. simulación → implementación en robot real):
Planeación → Configuración de Hardware → Desarrollo de Software → Simulación → Pruebas Físicas → Validación Final
- En la planeación, se define el enfoque del sistema, se establecen los requerimientos y organización del equipo.
- En la configuración de Hardware, se prepara el entorno físico de trabajo y se valida el funcionamiento de componentes esenciales.
- En el desarrollo de software, se implementa y valida los algoritmos de navegación, percepción y localización.
- En la simulación, se valida el comportamiento del sistema en simulación.
- En las pruebas físicas, se prueba en la pista física el comportamiento del robot.

## 2. 🤖 Especificaciones del Robot

### 2.1 En Simulación
- **Nombre del Robot**: Puzzlebot Jetson Lidar Edition
- **Tipo de plataforma**: Robot de tracción diferencial. Con una bola giratoria (caster ball). Su movimiento se basa en dos ruedas accionadas por separado, ubicadas a ambos lados del chasis. Puede cambiar de dirección variando la velocidad relativa de rotación de sus ruedas.
- **Dimensiones (simuladas)**:
  - base_link 5 cm en z respecto a base_footprint.
  - wheel_left_link 5.2 cm en x, 9.5 cm en y respecto a base_link.
  - wheel_right_link 5.2 cm en x, -9.5 cm en y respecto a base_link.
  - lidar_base_link 4.25 cm en x, 7 cm en z respecto a base_link.
  - laser_frame 3.5 cm en z, -pi en yaw respecto a lidar_base_link. 
- **Plugins usados**: libDiffDynamicPlugin.so
- **Sensores Simulados**: 
  - LiDAR:
    - Modelo: gpu_lidar.
    - Rango: 5 cm a 12 m.
    - Campo de visión: 360°.
    - frame_id: laser_frame.
    - Tópico: scan.
  - Cámara:
    - Modelo: N/A.
    - Resolución: 640 x 480 pixeles.
    - Tópico: camera.
  - Otros sensores: N/A.
- **Archivos URDF/XACRO**:
  - puzzlebot_jetson_lidar_ed.xacro: define el robot completo.
  - puzzlebot_jetson_lidar_base.urdf.xacro: define la base del robot.
  - parameters_jetson_lidar_ed.xacro: define las medidas de la base del robot.
  - materials.xacro: define las propiedades de los materiales del robot.
  - macros.xacro: define la física del robot.
  - laser_camera_bracket.xacro: define la cámara del robot.
  - lidar.xacro: define el LiDAR del robot.
  - gazebo_control.xacro: define los plugins de Gazebo Garden que realizan acciones como publicar velocidades de las ruedas, estado de las articulaciones, transformaciones de los marcos de referencia del robot, activar los sensores.
- **Entorno simulado**: Archivo .world. Diseñado en Gazebo Garden. Incluye paredes y obstáculos.

### 2.2 En Robot Físico
- **Nombre del Robot**: Puzzlebot Jetson Lidar Edition
- **Fabricante**: Manchester Robotics Ltd.
- **Tipo de plataforma**: igual que en simulación.
- **Radio de Rueda (en metros)**: 0.05.
- **Separación entre Ruedas (en metros)**: 0.18 aproximadamente.
- **Tipo de tracción**: diferencial.
- **Sensores Físicos**:
  - LiDAR físico:
    - Modelo: RP LiDAR A1
    - Puerto de conexión: ttyUSB0 o ttyUSB1
    - frame_id: laser (predeterminado)
  - Cámara física:
    - Raspberry Pi
    - Tipo de interfaz: CSI
    - Resolución: 1280 x 960 pixeles (predeterminado)
    - Nodo ROS usado: video_source/video_source
  - Otros sensores físicos: Encoder de ruedas.
- **Controladores**:
  - Microcontrolador: Hackerboard.
  - Computadora principal: Jetson Nano, fuente de alimentación INIU Power Bank USB C 10000mAh.
- **Sistema Operativo**:
  - OS del Jetson: Linux

---

## 3. 🧰 Stack de Software

- **Distribución de ROS 2**: 
 - Ros Humble Hawksbill, instalada tanto en el entorno de simulación (PC) como en la computadora Jetson Nano del robot físico. 
- **Entorno de Simulación**: 
 - Gazebo Garden. Se utiliza para probar el comportamiento del robot en entornos con obstáculos personalizados mediante un archivo .world. El entorno incluye simulación de sensores como LiDar y controladores diferenciales, además de plugins de física para simular la cinemática del robot. 
- **Frameworks / Stacks usados**:
 - Nav2: Para la navegación autónoma, incluye los nodos de planificación global, local, recuperación y controladores como DWB (Dynamic Window Approach).
 - micro-ROS: Utilizado para la comunicación eficiente entre el microcontrolador del robot (Hackerboard) y Ros 2, permite enviar datos como velocidades decodificadas y recibir comandos de velocidad. Corre el agente micro-Ros en la Jetson Nano, y el firmware del microcontrolador corre en FreeRTOS o Arduino. 
 - SLAM Toolbox: Utilizado para la construcción del mapa en tiempo real (modo online_sync), y para guardar y reutilizar mapas en modo localization.
 - AMCL (Adaptive Monte Carlo Localization): Se emplea en fases posteriores del proyecto para localización basada en mapas previamente generados. Es una alternativa a SLAM Toolbox en modo localización. 
 - TF2: Se usa para manejar las transformaciones entre los marcos de referencia del robot (como base_link, odom, map, laser_frame, etc.)
 - Lifecycle Manager: Controla el estado de los nodos de Nav2 y SLAM Toolbox (configuración, activación, desactivación, etc.).
 - RViz2: Visualizador para mostrar el mapa, la posición estimada del robot, trayectorias planeadas y sensores como el LiDAR. 
 - ros2_controllers: Controlador diferencial utilizado para convertir los comandos de velocidad (cmd_vel) en velocidades individuales por cada rueda.
 - ¿Otros?
  - `robot_state_publisher`
  - `joint_state_publisher_gui`
  - `slam_toolbox`, `nav2_bringup`
  - `teleop_twist_keyboard` o `joy_teleop`

- **Nodos personalizados creados**
| Nombre del Nodo            | Funcionalidad                                                                                       | Integración                                                                 |
|----------------------------|-----------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------|
| `puzzlebot_kinematic_model`| Convierte `cmd_vel` en velocidades individuales de ruedas. Pública odometría opcionalmente.         | Input: `cmd_vel`. Output: `wheel_vel_left`, `wheel_vel_right`.              |
| `puzzlebot_physical_sim`   | Simula movimiento físico con encoders, pública odometría, TF y estados de ruedas.                  | Reemplazable por datos físicos.                                             |
| `puzzlebot_controller`     | Controlador que guía al robot hacia la meta.                                                        | Subs: `pose`. Publ: `cmd_vel`.                                              |
| `localisation_node`        | Estima la posición del robot con odometría.                   | Fuente de `tf` y `odom`.                                           |

- **Nodos clave que se ejecutan** (en Jetson o PC):
| Nodo                     | Ubicación | Función Principal                                                 | Pública                                             | Se suscribe a                                      |
|--------------------------|-----------|-------------------------------------------------------------------|-----------------------------------------------------|----------------------------------------------------|
| `nav2_bringup`           | Jetson    | Maneja navegación con Nav2                                        | `cmd_vel`, `plan`, `costmaps`                       | `scan`, `odom`, `map`                              |
| `slam_toolbox`           | Jetson    | SLAM en línea o localización con mapas guardados                  | `map`, `pose`, `tf`                                 | `scan`, `odom`                                     |
| `micro_ros_agent`        | Jetson    | Comunicación serial con microcontrolador vía micro-ROS            | `velocity_enc_l`, `velocity_enc_r`, `imu`, etc.     | `cmd_vel`, otros comandos                           |
| `robot_state_publisher`  | Jetson/PC | Publica transformaciones entre enlaces del robot (`/tf`, etc.)    | `tf`, `tf_static`                                   | `joint_states`                                     |
| `rviz2`                  | PC        | Visualización en tiempo real                                      | —                                                   | `map`, `odom`, `scan`, `tf`, `goal_pose`           |
| `controller_server`      | Jetson    | Controlador local (Nav2)                                          | `cmd_vel`                                           | `plan`, `odom`, `local_costmap`                    |
| `planner_server`         | Jetson    | Planificador global (Nav2)                                        | `plan`                                              | `goal_pose`, `global_costmap`                      |
| `amcl` *(opcional)*      | Jetson    | Localización probabilística                                       | `amcl_pose`, `pose`                                 | `scan`, `map`, `initial_pose`                      |

 - ¿Cómo se lanza el agente micro-ROS?: ros2 launch puzzlebot_ros micro_ros_agent.launch.py
 - ¿Qué tópicos produce y consume cada nodo?
  - Nav2:
   - Consume: scan, odom, map, goal_pose.
   - Publica: cmd_vel, plan, costmaps.
  - SLAM / AMCL:
   - Consume: scan, odom, initial_pose.
   - Publica: map, pose, tf.
  - Micro-Ros:
   - Publica las velocidades encodificadas y sensores.
   - Recibe comandos de velocidad desde cmd_vel.

---

## 4. 🗂 Archivos de Configuración

### 4.1 Contenido Relevante en `.bashrc`
```
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
```

### 4.2 Árbol de Transformaciones TF
- Árbol esperado: `map → odom → base_footprint → base_link → lidar_base_link → laser_frame`, `map → odom → base_footprint → base_link → wheel_left_link`, `map → odom → base_footprint → base_link → wheel_right_link`
- Transformaciones estáticas necesarias: `map → odom`
- Errores comunes y solución: Si en la terminal o en RViz hay errores de TF, o alguna advertencia, o el robot en RViz oscila entre 2 posiciones, la solución consiste en verificar que map sea el primer frame, verificar qué frames se publican en gazebo_control.xacro, no publicar el mismo frame a la vez (si se crea un nodo para publicar un frame, no debe publicarse en gazebo_control.xacro).

---

## 5. 🔄 Integración con Proyecto de Manchester

- ¿Qué publicaban los nodos originales de Manchester? ¿Tenían prefijos?: El archivo launch gazebo_example_launch.py de Manchester del paquete puzzlebot_gazebo lanzaba únicamente los nodos ros_gz_image, ros_gz_bridge y robot_state_publisher. ros_gz_bridge se suscribe a cmd_vel, que debe ser publicado corriendo teleop_twist_keyboard. ros_gz_bridge publica camera_info, VelocityEncL, VelocityEncR, ground_truth, joint_states, scan, tf y tof_scan. robot_state_publisher se suscribe a joint_states y publica robot_description, tf y /tf_static. 
- ¿Qué tópicos crearon tus propios nodos? ¿Con qué tipo de mensajes?: cmd_vel tipo Twist del nodo de control, robot_pose tipo Odometry del nodo de localización, y goal tipo Point en el controlador.
- ¿Qué eliminaste de Gazebo para no duplicar funcionalidades?: el plugin de gz-sim-diff-drive-system y el de gz-sim-odometry-publisher-system

---

## 6. ❗ Retos Frecuentes por Categoría

### 6.1 En Simulación
Describe problemas que enfrentaste en simulación:
- Problema 1: El robot colisionaba con las paredes intentando atravesarlas. En consola marcaba que el robot seguía intentando navegar a la coordenada seleccionada sin embargo en RViz se podía ver que ya no se movía y en Gazebo él como había colisionado o atravesado las paredes. Los pasos que seguimos para poder solucionarlos fueron: volver a mapear de cero ya que es muy común que al momento de recorrer el laberinto existan zonas que no queden escaneadas a la perfección. Segundo, probar el nuevo mundo y si sigue sin funcionar pasar al tercer paso. Tercer paso, modificar las variables en el .yaml hasta que el robot cuente con la distancia tolerante con las paredes y deje de colisionar. 
- El robot no se visualizaba correctamente en RViz, o los datos de SLAM Toolbox parecían inconsistentes.
- Se verificó que el parámetro use_sim_time estuviera activado tanto en los archivos de lanzamiento como en los YAML utilizados por SLAM Toolbox y Nav2. También se confirmó que Gazebo estuviera publicando en el tópico /clock, ya que de no hacerlo, los nodos que dependen del tiempo simulado quedarían congelados o con comportamiento errático. Además, se revisó el parámetro frame_id del LiDAR para asegurar que coincidiera con el definido dentro del árbol TF. Se solucionaron errores de visualización reiniciando RViz con la configuración correcta de fixed_frame en map o odom según el estado de localización.


### 6.2 En Robot Físico
Problemas reales que enfrentaste:
- Error 1: El nombre del tópico del LiDar no coincide por lo tanto no se ejecutaba. Cambiarle el nombre a las transformadas de la simulación para que empataron con las del robot físico. 
- Error 2: Los mensajes no se estaban recibiendo correctamente debido a que la Jetson Nano y la PC tenían diferente fecha y hora. Entrar a la Jetson Nano y establecerla en la misma zona horaria que la PC. Confirmar con el comando de timedatectl que el desfase de tiempo no sea muy grande. 
- Error 3: Cable Flex de cámara dañado. La cámara dejó de publicar imágenes de manera repentina durante las pruebas físicas, incluso tras reiniciar nodos y revisar el topic con rqt_image_view. Se inspeccionó físicamente la conexión de la cámara, notando que el flex cable estaba excesivamente caliente. Se determinó que el calor del Jetson Nano sobrecargó el cable. Se reemplazó por un flex cable nuevo en buen estado, lo que restauró la transmisión de imagen.
- Error 4: Pérdida de conexión SSH. La red Wi-Fi creada para establecer comunicación entre la Jetson y la laptop desaparecía intermitentemente, impidiendo el acceso vía SSH. Se intentó reiniciar los servicios de red, pero el problema persistía. Como solución alternativa, se activó el hotspot de una computadora para mantener una red estable y persistente entre dispositivos.
- Error 6: Comportamiento inestable del Robot. El Puzzlebot se movía de forma agresiva, retrocedía bruscamente y giraba en su propio eje al detectar paredes. En ocasiones quedaba atascado en bucles de giro. Se revisó la lógica de los algoritmos, se ajustaron las ganancias de control y se redujeron las velocidades lineales y angulares. Esto estabilizó el comportamiento y permitió una navegación más fluida.
- Error 7: El scan del LiDAR en RViz está volteado de atrás hacia adelante del robot. Esto se resolvió cambiando el ángulo de yaw del frame del LiDAR en el archivo lidar.xacro aumentando +pi el ángulo . La línea que se sustituyó fue lidar_base_pos_w:='0.0' por lidar_base_pos_w:='3.14159'.
- Error 8: La distancia entre las ruedas del robot es aproximadamente 0.18 m en el robot físico. Por lo tanto, en la versión física del proyecto es preferible modificar el archivo de parameters_jetson_lidar_ed.xacro sustituyendo <xacro:property name="wheel_base_distance" value="0.19"/> por <xacro:property name="wheel_base_distance" value="0.18"/>
---

## 7. 🚧 Malentendidos Comunes

Lista de conceptos que tú (o tu equipo) asumieron erróneamente y que resultaron ser diferentes:
- Se hizo la suposición que con cmd_vel el robot avanzaría suavemente. Resultó ser muy agresivo, fue necesario calibrar velocidades y ganancias, ya que no requiere un valor tan alto ya que en ocasiones el robot se llega a detener y marcar un error, por usar un valor de velocidad alta.
- Creer que basta lanzar Nav2 sin revisar costmaps y controladores en YAML. En realidad, los archivos YAML como nav2_params.yaml definen parámetros críticos para el comportamiento del planificador global, el controlador local, y los mapas de costos (costmaps). Si no se configuran correctamente variables como el tamaño del global_costmap, la resolución, los sensores utilizados (observation_sources), o el tipo de planner (GridBased, Navfn, etc.), el robot puede fallar al evitar obstáculos o al trazar rutas óptimas.
- Asumimos que nuestro Puzzlebot físico en su tópico scan (del sensor RP LiDAR A1) iba a publicarse con 360 valores en el campo ranges, como el lidar predeterminado de Manchester en simulación. Sin embargo, el lidar físico publicaba el tópico scan con 1080 valores en el campo ranges. 
- Asumimos que el LiDAR físico estaba sincronizado con el reloj de la PC donde corría ROS 2, pero no era así. Esta desincronización provocaba un desfase temporal en los mensajes del tópico /scan, lo cual generaba errores en la localización y navegación, ya que los datos del sensor llegaban con retraso respecto al resto del sistema.
---

## 8. 📸 Recursos Visuales

Lista de imágenes que acompañan tu archivo:
- ‘Diagram_Puzzlebot.png’: diagrama de conexión del Puzzlebot.
- ‘node_diagram_mcr.png’: diagrama de nodos del paquete de Manchester predeterminado.
- ‘tf_diagram_mcr.png’: diagrama de TF del paquete de Manchester predeterminado.
- ‘node_diagram.png’: diagrama de nodos del paquete modificado para proyecto en simulación.
- ‘tf_diagram.png’: diagrama de TF del paquete modificado para proyecto en simulación.
- ‘node_diagram_physical.png’: diagrama de nodos del paquete modificado para proyecto en físico.
- ‘tf_diagram_physical.png’: diagrama de TF del paquete modificado para proyecto en físico.
---

## 9. 🪜 Pasos del Proyecto (Timeline)

Lista ordenada de pasos que seguiste desde la simulación hasta la ejecución real:

1. Crear repositorio en GitHub
- De preferencia que solo esté tu paquete de ROS2 para evitar compilarlo mal, trabajen en ramas es más organizado y evita problemas.
2. Integrar el proyecto base de navegación de Turtlebot3 al robot Puzzlebot
- Te recomiendo usar las plantillas de manchester e ir adaptándolas para que funcionen haciendo uso del nav2. Puedes ocupar los nodos que te dio tu profesor para hacer esto. 
3. Diseñar el entorno (laberinto)
- Este paso es de lo más tedioso sobre todo por las medidas, puedes hacer uso de otro software aparte de Gazebo para diseñar y modelar el laberinto, solo toma en consideración que el archivo final debe de ser un STL Binario, de otra forma no funcionara con gazebo.
4. Implementar el stack de navegación en simulación
- Te recomiendo hacer uso de la plantilla que proporciona el profesor Juan Ma, la mayoría de las veces los errores no son por el código sino por el archivo de parámetros. Te recomiendo hacer una copia de la plantilla e ir experimentando con esta Considera que la estructura de tu proyecto debe de tener al menos 3 launch, uno principal y otros dos para decidir entre navegar y mapear. Te recomiendo empezar por el de mapeo, de otra forma no tendrás mapa para navegar. Cuando quieras generar el mapa considera tener listo el comando para guardarlo, y para navegar asegúrate de que la carpeta del mapa esté incluida en el setup.py . Para mapear te recomiendo hacer uso del teleoptwistkeboard de ros2, y hacerlo a muy baja velocidad para poder capturar la mayor cantidad de detalle. Si el profesor te proporciona una plantilla para estos launch files, no hay muchos cambios que debas de hacer.
5. Afinar parámetros de Nav2 (planner, controller)
- Prueba con distintos parámetros, y si tienes alguna duda acerca de alguno buscalo en internet, generalmente ajustar los mapas locales y globales soluciona la mayoría de problemas con el Nav2
6. Configurar robot físico (pasos detallados: micro-ROS, URDF, etc.)
- Haz uso de un HotSpot, windows de preferencia, en ubuntu se puede hacer pero requieren un adaptador de red extra, esto te ayuda a eliminar mucha de la latencia entre el puzzle bot y la computadora y corrige la hora del sistema operativo del robot, esto es especialmente importante para poder mover el robot. Considera que este hotspot no debe de estar haciendo uso de internet para mejor funcionamiento. Te recomendamos hacer uso del mismo mapa que usaste en simulación, así te evitas tener que volver a mapear.
7. Ajustar TFs, tópicos y controladores reales
- Considera que la simulación de manchester pública las transformaciones del robot haciendo uso de gazebo, de modo que sin gazebo tendrás que hacer uso de un propio JoinStatePublisher que publique la posición inicial del robot y actualice su posición y odometría también la rotación de las ruedas, aunque esto último es estético pero puede ser útil para debuggear ciertos problemas. 
8. Validar movimiento autónomo en entorno físico
- Ten cuidado, aunque el Nav2 evita que el robot llegue a golpearse con las paredes bruscamente no está de más que un compañero cuide del robot. Por otro lado considera tunear principalmente las velocidades del robot, puesto que este tiene muchas más limitaciones que el robot simulado, experimentar con los diferentes valores es lo mejor que puedes hacer. También considera que mientras más larga la ruta más fácil será que el robot se pierda, por lo que intenta rutas cortas al inicio y luego trata de tunearlo para rutas largas.

---

## 10. 🔗 Enlaces Útiles

Incluye todas las páginas, videos, foros, tutoriales que consultaste:
- https://navigation.ros.org/
- https://github.com/SteveMacenski/slam_toolbox
- https://wiki.ros.org/tf2
- https://www.theconstructsim.com/
- https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/ 

## Incluir cualquier otra sección que en su experiencia consideren relevante. Algo que les hubiera gustado haber sabido en un inicio y que les hubiera hecho el desarrollo del proyecto más sencillo. 


