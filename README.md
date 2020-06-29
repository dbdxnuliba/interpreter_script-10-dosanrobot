//BRAZO ROBÓTICO M1013 DE LA EMPRESA DOOSAN ROBOTICS
//Los programas se realizaron de manera que el robot fuera manipulado con un  control de Ps4

1. Crear un espacio de trabajo ROS

  cd ~
  mkdir -p workspace_doosan/src
  cd ..
  catkin_make

2. clonar el repositorio con los archivos para visualizar y manipular el doosan-robot m1013

  git clone https://github.com/wilderarias95/practica_robotica.git

3.  Visualización en Gazebo

  roslaunch dsr_launcher single_robot_gazebo.launch model:=m1013 gripper:=robotiq_2f 

NOTA: este modelo del gripper contiene errores, consultar en el siguiente repositorio si ya se realizaron las correciones de los problemas https://github.com/doosan-robotics/doosan-robot/issues/52

4. Visulazación en Rviz

  roslaunch dsr_launcher single_robot_rviz.launch model:=m1013 gripper:=robotiq_2f

5. Ejecutar script para manejar las ordenes del control de Ps4

  rosrun dsr_example_py joy_ds4.py

6. Manejo del control Ps4

  Movimiento en Z:  el movimiento en Z se da con el joystick derecho, con movimientos hacía arriba y hacía abajo

  Movimiento en X: el movimiento en X se da con el joystick izquierdo, con movimientos hacía arriba y hacía abajo

  Movimiento en Y: el movimiento en Y se da con el joystick izquierdo, con movimientos hacía la izquierda y hacía la derecha

  Abrir la pinza: Se pulsa el botón triangulo

  Cerrar la pinza: Se pulsa el botón circulo

  recoger el objeto: Se pulsa el botón X