package frc.robot.subsystems;

import static frc.robot.Constants.*;

public class JointSubsystem extends SingleAxisSubsystem{

    public JointSubsystem() {
        super(JOINT_ZERO_SWITCH_CHANNEL, JOINT_ZERO_SWITCH_CHANNEL, "Joint");
        setPID(1.0e-1, 1.0e-6, 1.0);
    }
    
}
