package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class TelescopeSubsystem extends SubsystemBase{

    private final CANSparkMax telescopeMotor = new CANSparkMax(TELESCOPE_MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput zeroLimitSwitch = new DigitalInput(TELESCOPE_ZERO_SWITCH_CHANNEL);
    private final SparkMaxPIDController pidController = telescopeMotor.getPIDController();
    private final RelativeEncoder encoder = telescopeMotor.getEncoder();

    private double zeroPosition = 0.0;
    private double lowNodePosition = 0.0;
    private double midNodePosition = 0.0;
    private double highNodePosition = 0.0;
    private double shelfPosition = 0.0;

    public void setTelescopePercent(double percent) {
        pidController.setReference(percent, ControlType.kDutyCycle);
    }

    public void setTelescopePosition(double position) {
        pidController.setReference(position + zeroPosition, ControlType.kPosition);
    }

    public Command runLowNodeCommand() {
        return this.runOnce(() -> setTelescopePosition(lowNodePosition));
    }

    public Command runMidNodeCommand() {
        return this.runOnce(() -> setTelescopePosition(midNodePosition));
    }

    public Command runHighNodeCommand() {
        return this.runOnce(() -> setTelescopePosition(highNodePosition));
    }

    public Command runShelfCommand() {
        return this.runOnce(() -> setTelescopePosition(shelfPosition));
    }

    public Command runZeroCommand() {
        return this.runOnce(() -> setTelescopePosition(0));
    }

    @Override
    public void periodic() {
        if(zeroLimitSwitch.get()) {
            pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
            zeroPosition = encoder.getPosition();
        }
    }
    
}
