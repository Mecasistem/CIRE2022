# Concurso Iberoamericano de Robótica Espacial 2022
Programa Espacial Universitario - UNAM

Canal de Youtube del equipo Mecasistem: [Mecasistem CIRE 2022](https://www.youtube.com/@mecasistemconace2219) 

## Requerimientos

* Ubuntu 20.04
* ROS Noetic http://wiki.ros.org/noetic/Installation/Ubuntu

## Máquina Virtual

Se puede descargar una máquina virtual para [VirtualBox](https://www.virtualbox.org/wiki/Downloads) con todo lo necesario ya instalado de [esta dirección.](https://drive.google.com/drive/folders/1DYhmegVFEz7VA69uncpYsL8Ck0HbaIEz?usp=sharing) <br>
En esa misma carpeta hay un video con instrucciones para usar la máquina virtual. <br>
Se recomienda configurar la máquina virtual con 4 CPUs y 4GB de RAM.
Usuario: cire2022 <br>
Contraseña: cire2022

## Prueba

Abra una terminal y ejecute los siguientes comandos:

* $ cd CIRE2022
* $ git pull
* $ cd catkin_ws
* $ catkin_make
* $ source devel/setup.bash
* $ roslaunch bring_up stage02.launch

Se debería ver un RViz como el siguiente:

<img src="https://github.com/mnegretev/CIRE2022/blob/master/Media/rviz.png" alt="RViz" width="639"/>

Y un Gazebo como el siguiente:

<img src="https://github.com/mnegretev/CIRE2022/blob/master/Media/gazebo.png" alt="Gazebo" width="631"/>

## Instalación nativa

Para utilizar una instalación nativa, contactar a los organizadores. 

## Contacto
Dr. Marco Negrete<br>
Profesor Asociado C<br>
Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
[mnegretev.info](http://mnegretev.info)<br>
marco.negrete@ingenieria.unam.edu<br>

## Ubicación de los archivos
* La solucion del problema de la etapa 4 se encuentra dentro de la siguiente carpeta: CIRE2022/catkin_ws/src/ 
 En el paquete stage04_mv dentro de los scrips se encuentra stage04_mv.launch que es el archivo .launch que ejecuta todos los nodos de los paquetes de ros y el archivo stage04_mv_node-py para realizar la exploracion autonoma del entorno del robot takeshi 

 Para poder ejecutar el programa se ocupan los paquetes de ros de explore_lite y move_base asi como sus dependencias 

 Para comodidad se puede ejecutar el siguiente comando para instalar de una sola vez las dependencias 

 * $ rosdep install --from-paths src --ignore-src -r -y

