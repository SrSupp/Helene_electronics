Author: Sven Suppelt		Date: 22.08.2020
<a href="url"><img src="pictures/title.png" width="100%"></a>

# HELENE - electronics for the low cost 6DOF robot
This repository is responsible for the electronics for the Helene robotarm. Included are the PCBs and the program to run on these. 

The robot arm has a motor control board on each of its 6 axes. The board of the lowest axis, the 1st axis, acts as ROS Serial Slave. All other axes receive their control data from the 1st axis via CAN. 

A single motor control board consists of a microcontroller (ESP32), a stepper motor driver (TMC5160), a magnetic encoder to determine the current position of the robot arm (AS5048) and a CAN transceiver. 
The 1st axis is equipped with an external magnetic encoder due to lack of space. All other axes have enough space so that the board can be mounted directly above the required magnet. 

# Folder structure
The boards are located in the folder "pcb". Here you will find the board for the external encoder and the motor control board. The source code for the boards can be found in the "code" folder. Stored there are the PlatformIO projects for the ROS Serial Slave (LCR_ESP_Master) and the other boards connected via CAN (LCR_ESP_Slave).

# System structure of the robot arm
 <a href="url"><img src="pictures/system.PNG" width="100%"></a>

# Available Serial Commands
Each slave is equipped with the possibility of serial communication. Via this interface the calibration value of each axis can be read and set and status information can be read out. It is absolutely necessary to calibrate each axis individually!
The following commands are available:

long form | short form | description
-------- | -------- | --------
*IDN? | IDN? | Prints identification
HELP? | HELP | Prints a help screen (not yet implemented)
CALSINGLEJOINT [joint ID] | CAJO [joint ID] | starts the calibration process of given joint. Instructions will be printed after that command!
SETNULL [0,1] | SENU [0,1] | globally enables (1) or disables (0) the Nullpunktfahrt at startup
GETCALVALUE [joint ID] | GECA [joint ID] | Prints the current saved calibration value of given joint
SETCALVALUE [joint ID] | SECA [joint ID] | Sets the current saved calibration value of given joint. Instructions are printed after that command!
GETACTUAL [joint ID] | GEAC [joint ID] | Prints the actual angle of given joint. The actual angle is: Raw_Angle - Calibration_Value
GETRAW [joint ID] | GERA [joint ID] | Prints the raw angle of given joint
DONULL [joint ID] | DONU [joint ID] | The given joint will do a Nullpunktfahrt
OFFANDNULL [joint ID] | OFNU [joint ID] | Sets a offset value on the current calibration value. After that a Nullpunktfahrt is initiated. Instructions are printed out!
DIAG | DIA | Prints out diagnose data


