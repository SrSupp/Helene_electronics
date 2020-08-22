Author: Sven Suppelt		Date: 22.08.2020
<a href="url"><img src="pictures/title.png" width="100%"></a>

# HELENE - electronics for the low cost 6DOF robot
This repository is responsible for the electonics for the Helene robotarm. Included are the PCBs and the program to run on these. 

The robot arm has a motor control board on each of its 6 axes. The board of the lowest axis, the 1st axis, acts as ROS Serial Slave. All other axes receive their control data from the 1st axis via CAN. 

A single motor control board consists of a microcontroller (ESP32), a stepper motor driver (TMC5160), a magnetic encoder to determine the current position of the robot arm (AS5048) and a CAN transceiver. 
The 1st axis is equipped with an external magnetic encoder due to lack of space. All other axes have enough space so that the board can be mounted directly above the required magnet. 
