# RC Simulation Description

Paquete de descripcion para un auto RC sencillo en ROS 2 con Gazebo (gz). Instrucciones pensadas para WSL2 en Windows con Ubuntu 24.04 (Noble) y ROS Jazzy.

## Estructura del paquete
- `urdf/`: modelo `rc_car.urdf.xacro` con llantas delanteras direccionables.
- `launch/`: `spawn_rc_car.launch.py` levanta Gazebo (gz) y spawnea el robot.
- `worlds/`: `basic_track.world` con pista rectangular y marcador de flecha (planos y luz locales, sin dependencias de model://).
- `config/rviz/`: configuracion rapida para RViz (opcional).
- `meshes/`: lugar para mallas personalizadas si se necesitan.

## Instalacion paso a paso (WSL2, Ubuntu 24.04, ROS Jazzy)
1) Preparar sistema y herramientas base:
```
sudo apt update
sudo apt install -y curl gnupg2 lsb-release software-properties-common \
  build-essential git python3-colcon-common-extensions python3-vcstool
```

2) Agregar repositorio de ROS 2 Jazzy:
```
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
```

3) Instalar ROS 2 desktop y deps que usamos:
```
sudo apt install -y ros-jazzy-desktop \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher ros-jazzy-xacro
```

4) Instalar rosdep (en Noble el paquete es `python3-rosdep`), inicializar y actualizar:
```
sudo apt install -y python3-rosdep
sudo rosdep init        # solo la primera vez
rosdep update
```

5) (Opcional) Verificar que `gz` esta en el PATH:
```
gz --help
```

6) Clonar o colocar este workspace y compilar:
```
cd ~/AiAtonomousRc
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Ejecucion
Lanza Gazebo y spawnea el auto:
```
ros2 launch rc_sim_description spawn_rc_car.launch.py z:=0.15
```

Argumentos utiles:
- `x`, `y`, `z`: posicion inicial del auto (z > 0 evita colisiones al spawnear).
- `world`: ruta al mundo SDF (por defecto `worlds/basic_track.world`).
- `rviz:=true` para abrir RViz con el modelo (por defecto `false`, solo Gazebo).

## Notas para WSL
- Usa WSLg o un servidor X/Wayland para ver la GUI de Gazebo. En Windows 11 con WSLg suele funcionar sin configuracion extra.
- Si cambias a otra distro ROS o Gazebo, ajusta los nombres de paquete en la seccion de instalacion.
