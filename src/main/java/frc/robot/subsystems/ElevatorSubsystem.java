package frc.robot.subsystems;

import static frc.robot.Constants.*;

public class ElevatorSubsystem extends SingleAxisSubsystem{

    public ElevatorSubsystem() {
        super(ELEVATOR_MOTOR_ID, ELEVATOR_ZERO_SWITCH_CHANNEL, "Elevator");
        setPID(1.0e-1, 1.0e-6, 1.0);
    }

}