# Prácticas 

Este es el repositorio oficial de las prácticas y trabajo de final de grado de Joel Ramos Beltrán.

Para llevar a cabo este proyecto, se ha usado material de los siguientes repositorios de Robotnik:

[Robotnik_description](https://github.com/RobotnikAutomation/robotnik_description/tree/humble-devel)

[Robotnik_sensors](https://github.com/RobotnikAutomation/robotnik_sensors/tree/humble-devel)

[Robotnik_simulation](https://github.com/RobotnikAutomation/robotnik_simulation/tree/humble-devel)

En este repositorio se encuentra el workspace de ROS2 con todo el trabajo, y dos documentos PDF, los cuales son guías para la migración y ejecución de Ignition y Webots a partir de mi trabajo.

Por el lado del Workspace, este se divide en varios paquetes. Los principales son:

- Robot_Description: Este paquete contiene la descripción de los robots en XACRO/URDF

- Robotnik_sensors: Similar al paquete de arriba, contiene la descripción de los sensores que utilizan los robots.

- Robotnik_simulation: Este es el más importante, ya que aquí se encuentra todo el material neceesario para la simulación tanto en Ignition como en Webots.

# Ejecución en Ignition

Abrimos 3 terminales en el directorio Robotnik_ws.

## Primera terminal (Lanzamiento del simulador):

Primero configuramos el entorno de ROS2 en esta terminal.

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Seguidamente, lanzamos el mundo con el siguiente comando:

```bash
ros2 launch robotnik_gazebo_ignition spawn_world.launch.py world:=prueba3
```

Este comando abre la simulación con el mundo "prueba3". Este parámetro se puede modificar en el caso de querer usar otro mundo.

## Segunda terminal (Lanzamiento del robot):

Primero configuramos el entorno de ROS2 en esta terminal.

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Seguidamente, lanzamos el robot con el siguiente comando:

```bash
ros2 launch robotnik_gazebo_ignition spawn_robot_dinamicbridge.launch.py robot:=rbwatcher namespace:=rambelrobot1 x:=1 y:=1
```

Este comando tiene varios parámetros que se pueden modificar para personalizar el robot que se va a lanzar:

- robot: Este es el modelo del robot. Además del "rbwatcher", se puede selccionar el "rbrobout".
- namespace: Este es el nombre propio que tendrá el robot lanzado. Servirá para diferenciarlo entre los demás robots con el mismo modelo.
- posición: Se puede modificar la x, y, z en la que el robot aparecerá en el mundo.

Tras lanzar el robot, además de aparecer este en el mundo, el rviz2 debería haberse abierto.

## Tercera terminal:

Tras lanzar el primer robot, en esta tercera terminal puedes lanzar un segundo robot (cuando el RTF de la simulación supere el 20%) y lo mismo si quieres lanzar un tercero o un cuarto. Además, esta terminal se puede usar para ejecutar comandos de ROS, como revisar los topics que se están publicando. 

Finalmente, esta terminal también se puede utilizar para conducir el robot lanzado:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/rambelrobot1/robotnik_base_controller/cmd_vel
```

# Ejecución en Webots

Abrimos 3 terminales en el directorio Robotnik_ws.

## Primera terminal (Lanzamiento del simulador):

Primero configuramos el entorno de ROS2 en esta terminal.

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Seguidamente, lanzamos el mundo con el siguiente comando:

```bash
ros2 launch webots_robotnik world_launch.py
```
En este caso, para cambiar el mundo que se lanzaría con el comando, habría que modificar el archivo de lanzamiento. Esto es debido a una función de Webots que impide pasarle argumentos de launchers.

## Segunda terminal (Lanzamiento del robot):

Primero configuramos el entorno de ROS2 en esta terminal.

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Seguidamente, lanzamos el robot con el siguiente comando:

```bash
ros2 launch webots_robotnik robot_launch_alfa.py robot:=rbwatcher namespace:=rbwatcher x:=2 y:=2 z:=0
```

Este comando tiene varios parámetros que se pueden modificar para personalizar el robot que se va a lanzar:

- robot: Este es el modelo del robot. Además del "rbwatcher", se puede selccionar el "rbrobout".
- namespace: Este es el nombre propio que tendrá el robot lanzado. Servirá para diferenciarlo entre los demás robots con el mismo modelo.
- posición: Se puede modificar la x, y, z en la que el robot aparecerá en el mundo.

Tras lanzar el robot, además de aparecer este en el mundo, el rviz2 debería haberse abierto.

## Tercera terminal:

Tras lanzar el primer robot, en esta tercera terminal puedes lanzar un segundo robot y lo mismo si quieres lanzar más en nuevas terminales. Además, esta terminal se puede usar para ejecutar comandos de ROS, como revisar los topics que se están publicando. 

Finalmente, esta terminal también se puede utilizar para conducir el robot lanzado:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/rambelrobot1/robotnik_base_controller/cmd_vel
```

# Navegación por marcadores ArUco

Para la navegación usando marcadores ArUco, se seleccionó Webots como el simulador principal en el que realizar las operaciones. Aún así, si modificas los topics del lanzador, es muy posible que funcione correctamente en Ignition y otros simualdores.

Abrimos 3 terminales en el directorio Robotnik_ws.

## Primera terminal (Lanzamiento del simulador):

Primero lanzamos Webots con un mundo creado para poner a prueba el sitema de visión.

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch webots_robotnik visual_world_launch.py
```


## Segunda terminal (Lanzamiento del robot):

Lanzamos el comando del robot con los parámetros exactos indicados. En este caso, al no haber por ahora argumentos para el ros2 run, el nodo está configurado para unps topics en específico, resultantes de lanzar el robot en Webots con el namespace "rbwatcher".

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch webots_robotnik robot_launch_alfa.py robot:=rbwatcher namespace:=rbwatcher x:=0 y:=0 z:=0
```

Tras lanzar el robot, además de aparecer este en el mundo preparado para la visión, el rviz2 debería haberse abierto.

## Tercera terminal:

En esta terminal será en la que se active el nodo de visión.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run webots_robotnik visual_node
```

Este nodo de visión, tras ejecutarse, abrirá una ventana con la visión de la cámrara delantera, el estado de la navegación y los marcadores ArUco detectados.

# Imágenes
   
Simulación en Ignition del RbRobout y RbWatcher

![](https://github.com/JoelRamosBeltran/Robotnik_practicas/blob/main/Fotos/Captura%20desde%202025-04-30%2009-21-08.png)

Simulación en Webots del RbRobout y RbWatcher

![](https://github.com/JoelRamosBeltran/Robotnik_practicas/blob/main/Fotos/Captura%20desde%202025-04-22%2013-52-41.png)

Mundo para pruebas de visión

![](https://github.com/JoelRamosBeltran/Robotnik_practicas/blob/main/Fotos/Captura%20desde%202025-06-02%2019-19-40.png)

Interfaz de visión (Por modificar=

![](https://github.com/JoelRamosBeltran/Robotnik_practicas/blob/main/Fotos/Captura%20desde%202025-06-02%2013-22-36.png)




