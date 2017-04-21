# Autonomous Mobile Robots Assignment 02

Implementation of the Braitenberg Vehicle 

## Major Tasks:
1. Implementation of differential drive mechanism using *differential_drive_emulator* node

2. Implementation of *braitenberg_vehicle* node to listen and respond to sonar readings
	* Abbility to  change the type (“A”, “B”, or “C”), and the connection factors of the vehicle

3. Test-run of the vehicle for each vehicle type, and description of the behavior of the robot

## Description of Behaviour:
###Type A:
* For this type of vehicle, the behaviour is such that the each sensor is directly connected to the corresponding wheel. That is, right sensor to right wheel and left sensor to left wheel.

* A test run was implemented with both factors as *1.0* and below were the observations:
	- The vehicle moved away from the obstacles.
	- Once any sensor detects the obstacle, it will make the wheel on that side turn faster and thus move to the opposite side.

###Type B:
* For this type of vehicle, the behaviour is such that the each sensor is connected to the opposite wheel. That is, right sensor to left wheel and left sensor to right wheel.

* A test run was implemented with both factors as *1.0* and below were the observations:
	- The vehicle moved towards the obstacles.
	- Once any sensor detects the obstacle, it will make the wheel on the opposite side turn faster and thus move towards the obstacle side.


###Type C:
* For this type of vehicle, the behaviour is such that the each sensor is connected to both of the wheels. 

* A test run was implemented with both factors as *1.0* and below were the observations:
	- The vehicle moved straight, irrespective of obstacle orientation.
	- Once any obstacle is detected, the vehicle will slow down gradually and finally crash into the obstacle
	- This happens because both the wheels get similar inputs from the sensors.

###Effect of *factors*
* The factor parameters set from the console directly affects the behaviour of the vehicle.  
* Since the factors are directly multiplied to the sensor readings before being sent to the wheels.
* Negative parameters of factors will make the vehicle move in reverse and thus will create behaviour that is unhandled in the above conditions