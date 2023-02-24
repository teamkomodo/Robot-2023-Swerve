package frc.robot.subsystems;

import static frc.robot.Constants.*;

public class ElevatorSubsystem extends SingleAxisSubsystem{

    private final double INCHES_PER_REVOLUTION = 3.53D/9.0D;

    public ElevatorSubsystem() {
        super(ELEVATOR_MOTOR_ID, ELEVATOR_ZERO_SWITCH_CHANNEL, "Elevator");
        encoder.setPositionConversionFactor(INCHES_PER_REVOLUTION);
        setPID(5.0e-2, 1.0e-6, 0.7);

        //Set node positions in inches above 0
        //These methods are required to set the positions, if the variables are directly assigned they will be overridden by the shuffleboard inputs
        setLowNodePosition(0);
        setMidNodePosition(16);
        setHighNodePosition(31);
        setShelfPosition(31);
        setMaxPosition(32);
    }

}