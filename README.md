# sequence_move_dynamixel
Algoritmo en C++ para la creación de secuencias en el Bogobot v2

## Descripción general
Programa para la ejecución de secuencias con el objetivo de crear movimientos predeterminados que el robots Bogobot v2 usará para cuando este se encuentre en juego, dichas secuencias se pueden crear a partir de teclas o la escritura de posición, así como existe la posibilidad de modificar una secuencia ya hecha, añadiendo, ejecutando o borrando una postura, por último este algoritmo puede leer dichas secuencias para conocer los valores de posición de cada uno de los motores Dynamixel Rx-28 y Rx-24f

## Pre-requisitos
El usuario deberá tener correctamente instalado ROS, GIT, así como la carpeta de `catkin_ws` inicializada así como los paquetes de Dynamixel para ROS, previamente instalado en el repositorio: [example_dynamixel](https://github.com/aaceves/example_dynamixel).

## Instalación
Para instalar el paquete de este repositorio llamado: `motionSeq`, se deben ejecutar en una terminal las siguientes instrucciones:

```
cd ~/catkin_ws/src
git clone https://github.com/JuanCarlos-MA/sequence_move_dynamixel.git
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

## Proceso de utilización

Al conectar el USB a la computadora con Ubuntu, es necesario verificar que el dispositivo aparece como algún puerto USB, p. ej. `/dev/ttyUSB0`, así como es de suma importancia habilitar los permisos de lectura de dicho dispositivo para ello se necesita escribir en una linea de comandos:
```
sudo chmod a+rw /dev/ttyUSB0 
```

Posteriormente deberá de abrir dos Terminales nuevas y ejecutar los siguientes comandos:

* Terminal 1
```
roscore
```
* Terminal 2

Para correr el programa:
```
rosrun motores motionSeq
```

## Diagrama de funcionamiento del programa
En este diagrama se puede observar el funcionamiento del programa en el cuál en cada una de las flechas se muestran las opciones válidas del menú y los submenús que tiene el algoritmo, es importante leerlo y analizarlo con detenimiento.

<p align="center">
  <img width="650" length="250" src="Anexos/motionS_diag.png">
</p>

## Autor

**Juan Carlos Martínez Aguilar** *Investigación en Robots Humanoides*
