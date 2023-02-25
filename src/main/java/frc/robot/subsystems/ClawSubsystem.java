package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class ClawSubsystem extends SubsystemBase{
    Compressor compressor = new Compressor(2, PneumaticsModuleType.REVPH);
    DoubleSolenoid solenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, CLAW_SOLENOID_FORWARD_CHANNEL, CLAW_SOLENOID_REVERSE_CHANNEL);

    public ClawSubsystem() {
        compressor.enableDigital();
    }

    public Command openCommand() {
        return this.runEnd(() -> solenoid.set(kReverse), () -> solenoid.set(kOff));
    }

    public Command closeCommand() {
        return this.runEnd(() -> solenoid.set(kForward), () -> solenoid.set(kOff));
    }

}
