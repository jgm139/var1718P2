# Práctica 2 - Mapeado 3D de interiores

>Por Robert Esclapez García y Julia García Martínez

Práctica 2 de la asignatura de Visión Artificial y Robótica.

**Compilación**
```bash
catkin_make
source devel/setup.bash
```


`load_model`
```bash
roslaunch load_model init.launch
```


`get_pointclouds`
```bash
rosrun get_pointclouds get_pointclouds_node
```


`process_environment`
```bash
rosrun process_environtment process_environtment_node
```


`navigation`
```bash
rosrun navigation navigation_node
```