package frc.robot.subsystems;

import static frc.robot.Constants.*;

public class TelescopeSubsystem extends SingleAxisSubsystem{

    public TelescopeSubsystem() {
        super(TELESCOPE_MOTOR_ID, TELESCOPE_ZERO_SWITCH_CHANNEL, "Telescope");
        setPID(1.0e-1, 1.0e-6, 1.0);
    }

}