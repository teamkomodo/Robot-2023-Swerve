# Commands Documentation

## Default Drive Command

Problem:  
Create a command that uses the Drivetrain subsystem in order to allow the driver to move the robot with a full ROM using both joysticks and an XBox controller

Driver Inputs:  
- Joystick with x, y, and rotation axis.
- XBox Controller joysticks

Expected Robot Motions:
- Robot moves left, right, forward, and backward with corresponding joystick inputs
- Robot rotates clockwise and counterclockwise with corresponding joystick inputs

Pseudo Code:
    
    function update() {
        Set translational speed of the robot to the values given by the input
        
        Set rotational speed of the robot to the values given by the input
    }

Testing Plan:
- Test code in simulation until robot moves in correct directions with approximatley correct speeds