## Elevator Subsystem

Actuators
- Motor for up/down movement

Sensors
- Zero limit switch
- Motor encoder

Shuffleboard Outputs  
- Motor speed
- Motor position

Inputs and Behavior
- Stow/Zero command -> zero at stow position
- Low Node command -> move to low node position
- Mid Node command -> move to mid node position
- High Node command -> move to high node position
- Joystick -> move up and down freely
- Zero limit switch active -> set zero position and stop motor

## Telescope Subsystem

Actuators
- Motor for extending telescope

Sensors
- Motor encoder
- Limit switch at zero position

Shuffleboard Outputs
- Motor speed
- Motor position

Inputs and Behavior
- Zero limit switch -> set zero position and stop motor
- Low Node Button -> moves telescope to low node position
- Mid Node Button -> moves telescope to mid node position
- High Node Button -> moves telescope to high node position
- Shelf Button -> moves telescope to shelf position
- R/L triggers -> manual control over telescope

## Joint Subsystem

Actuators
- Motor for rotating joint

Sensors
- Motor Encoder
- Limit switch at zero position

Inputs and Behavior
- Low node command -> move to low node position
- Mid node command -> move to mid node position
- High node command -> move to high node position
- Zero command -> move to zero position
- Zero limit switch active -> set zero position and stop motor

## Claw Subsystem

Mechanical Components
- Solenoid value for controlling 2 x pneumatic cylinders
- Compressor

Sensor Inputs
- None

Inputs and Behavior
- Close Button -> Closes claw
- Open Button -> Opens claw

## Lights Subsystem

Mechanical Components
- RGB controller

Behavior
- Flash Orange on button hold
- Flash Purple on button hold
