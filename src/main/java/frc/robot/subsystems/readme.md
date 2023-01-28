## Elevator Subsystem
Driver Inputs and Expected Results  
Right joystick Y axis controls elevator up/down

Sensor Inputs
- Upper limit switch
- Motor encoder

Shuffleboard Outputs  
- Motor speed
- Motor position

Sudocode:  
```
Elevator Subsystem Default Command:
    Set elevator motor speed to right joystick y axis

constant Motor Near End Max Speed

Elevator Subsystem.setElevatorSpeed( Input Speed )
{
    IF ( Top Limit Switch is activated )
        Set motor speed to 0
        exit function

    IF ( motor position is not near top or motor )
        Set motor speed to Input Speed
    ELSE
        Set motor speed to Motor Near Top Max Speed
}

Elevator Subsystem.periodic()
{
    Update shuffleboard values for motor speed and motor position
}
```