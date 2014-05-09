Traffic_lights_system_based_on_uCOSII
=====================================
PROJECT DESCRIPTION:
The purpose of the project was to create a traffic lights system using uCOSII real-time operating system
which runs on Windows OS. There are two cases as follows:

Case 1: Initially, there is one car and one pedestrian getting involved in the traffic
The longitudinal lane stands for car lane
The horizontal lane with red lines stands for pedestrian lane
Car moving pattern: Down -> Up -> Down -> Up
Pedestrian moving pattern: Right -> Left -> Right -> Left

The traffic lights are based on the real traffic lights systems implemented in Finland, with the traffic
lights for pedestrian lane having green light and red light and the traffic lights for car lane having
red light, yellow(amber) light, and green light.

	* Red light means stop for both pedestrians and cars
	* Yellow light means slowing down for cars in this project. In some countries, however, yellow light
	means accelerating the vehicle.
	* Green light means go for both pedestrians and cars.
	* Flashing green means slowing down for pedestrians. 

rtos1 folder contains the source code, configuration files, and the executable for case 1 of the traffic lights project.
C source file for this case is located at: Traffic_lights_systems_based_on_uCOSII/rtos1/BC45/SOURCE/TEST.C

Case 2: Initially, there are three cars and one pedestrian getting involved in the traffic
The three cars should be able to communicate with each other in order to come across the road
The longitudinal lane stands for car lane
The horizontal lane with red lines stands for pedestrian lane
Car moving pattern: Down -> Right -> Up -> Left -> Down
Pedestrian moving pattern: Right -> Left -> Right -> Left

The traffic lights are based on the real traffic lights systems implemented in Finland, with the traffic
lights for pedestrian lane having green light and red light and the traffic lights for car lane having
red light, yellow(amber) light, and green light.

	* Red light means stop for both pedestrians and cars
	* Yellow light means slowing down for cars in this project. In some countries, however, yellow light
	means accelerating the vehicle.
	* Green light means go for both pedestrians and cars.
	* Flashing green means slowing down for pedestrians. 

rtos2 folder contains the source code, configuration files, and the executable for case 2 of the traffic lights project.
C source file for this case is located at: /Traffic_lights_system_based_on_uCOSII/rtos2/BC45/SOURCE/TEST.C

USAGE:
This project was compiled using Borland compiler BC45, the executables run on 32-bit Windows OS. In this project,
They were executed on 32-bit Windows XP SP2 OS.
The executable for case 1 is located at Traffic_lights_system_based_on_uCOSII/rtos1/BC45/TEST/TEST.EXE
The executable for case 2 is located at Traffic_lights_system_based_on_uCOSII/rtos2/BC45/TEST/TEST.EXE
