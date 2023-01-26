package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PlaygroundSubsystem extends SubsystemBase{

    final CANSparkMax motor0 = new CANSparkMax(0, MotorType.kBrushless);
    final DigitalInput limitSwitch = new DigitalInput(1);
    
    public void setMotor0Speed(double speed) {
        if(limitSwitch.get()) {
            motor0.set(0);
            return;
        }
        motor0.set(MathUtil.clamp(speed, -1.0D, 1.0D));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
        SmartDashboard.putNumber("Motor0 RPM", motor0.getEncoder().getVelocity());
        SmartDashboard.putNumber("Motor0 position", motor0.getEncoder().getPosition());
    }
}
