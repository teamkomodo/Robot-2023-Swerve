package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase{
    
    private final CANSparkMax motor;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder encoder;
    
    private final ShuffleboardTab shuffleboardTab;

    private double p = 5.0e-2;
    private double i = 3.0e-6;
    private double d = 2.0;
    private double maxIAccum = 1.0e1;

    public IntakeSubsystem() {
        motor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setSmartCurrentLimit(30);
        
        encoder = motor.getEncoder();
        encoder.setPosition(0);

        pidController = motor.getPIDController();
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setIMaxAccum(maxIAccum, 0);
        pidController.setReference(0, ControlType.kDutyCycle);

        shuffleboardTab = Shuffleboard.getTab("Intake");

        shuffleboardTab.addDouble("Motor Velocity", () -> encoder.getVelocity());
    }

    public void setMotorDutyCycle(double dutyCycle) {
        pidController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

}
