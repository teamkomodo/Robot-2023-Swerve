package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase{
    
    private final CANSparkMax motor;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder encoder;
    private final DigitalInput distanceSensor;
    
    private final ShuffleboardTab shuffleboardTab;

    private double p = 1.0e-4;
    private double i = 1.0e-6;
    private double d = 4e-2;
    private double maxIAccum = 1.0e1;

    private double smoothCurrent = 0;
    private double filterConstant = 0.95;

    public IntakeSubsystem() {
        motor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setSmartCurrentLimit(30);
        //motor.setSmartCurrentLimit(3, 30);
        
        encoder = motor.getEncoder();
        encoder.setPosition(0);

        pidController = motor.getPIDController();
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setIMaxAccum(maxIAccum, 0);
        pidController.setReference(0, ControlType.kDutyCycle);

        distanceSensor = new DigitalInput(DISTANCE_SENSOR_PORT);

        shuffleboardTab = Shuffleboard.getTab("Intake");

        shuffleboardTab.addDouble("Motor Velocity", () -> encoder.getVelocity());
        shuffleboardTab.addDouble("Motor Current", () -> motor.getOutputCurrent());
        shuffleboardTab.addDouble("Smooth Current", () -> smoothCurrent);
        shuffleboardTab.addBoolean("Piece Detected", () -> pieceDetected());
    }

    @Override
    public void periodic() {
        smoothCurrent = smoothCurrent * filterConstant + motor.getOutputCurrent() * (1-filterConstant);
    }

    public void teleopInit() {
        pidController.setReference(0, ControlType.kDutyCycle);
    } 

    public void setMotorDutyCycle(double dutyCycle) {
        pidController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

    public void setMotorVelocity(double velocity) {
        pidController.setReference(velocity, ControlType.kVelocity);
    }

    public void holdMotorPosition() {
        pidController.setReference(encoder.getPosition(), ControlType.kPosition);
    }

    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    public double getSmoothCurrent() {
        return smoothCurrent;
    }

    public ShuffleboardTab getTab() {
        return shuffleboardTab;
    }

    public boolean pieceDetected() {
        return !distanceSensor.get();
    }

}
