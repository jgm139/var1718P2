# Práctica 2 - Mapeado 3D de interiores

>Por Robert Esclapez García y Julia García Martínez

Práctica 2 de la asignatura de Visión Artificial y Robótica.

**Compilación**
```bash
catkin_make
source devel/setup.bash
```

---

Nodo para navegar por la casa e ir haciendo el mapeado 3D.

`navigation`
```bash
rosrun navigation navigation_node
```

---

`load_model`
```bash
roslaunch load_model init.launch
```

---

`get_pointclouds`
```bash
rosrun get_pointclouds get_pointclouds_node
```

---

`process_environment`
```bash
rosrun process_environtment process_environtment_node
```

---

### Documentación
> Para mayor detalle de la implementación de la práctica, así como los experimentos, ver [VAR1718_P2.md](https://github.com/jgm139/var1718P2/blob/master/doc/VAR1718P2.md).