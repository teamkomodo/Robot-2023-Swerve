package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import java.util.function.BooleanSupplier;

public class ElevatorSubsystem extends SubsystemBase{
    
    // 9:1 reduction
    // 5.53 in circumference
    private final double INCHES_PER_REVOLUTION = 3.53D/9.0D;

    private final CANSparkMax motor;
    private final DigitalInput zeroLimitSwitch;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder encoder;

    private final ShuffleboardTab shuffleboardTab;

    private double p = 1.0;
    private double i = 1.0e-6;
    private double d = 0.7;

    private double SmartMotionMaxVel = 2000;
    private double SmartMotionMaxAccl = 1000;
    private double SmartMotionMinVel = 0;
    private double SmartMotionAllowedClosedLoopError = 1;

    private ControlType commadingControlType = ControlType.kSmartMotion;

    private boolean atLimitSwitch = false;
    private boolean atMaxLimit = false;
    private boolean atMinLimit = false;

    private int holdingCurrentLimit = 30;
    private int runningCurrentLimit = 60;

    private double commandedPosition = 0;

    private boolean useLimits = true;
    private boolean slowMode = false;

    private boolean zeroed = false;

    public ElevatorSubsystem(ShuffleboardTab mainTab) {
        
        zeroLimitSwitch = new DigitalInput(ELEVATOR_ZERO_SWITCH_CHANNEL);

        motor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setSmartCurrentLimit(holdingCurrentLimit, runningCurrentLimit);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(INCHES_PER_REVOLUTION);

        pidController = motor.getPIDController();
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setReference(encoder.getPosition(), commadingControlType);
        pidController.setSmartMotionMaxVelocity(SmartMotionMaxVel,0);
        pidController.setSmartMotionMaxAccel(SmartMotionMaxAccl, 0);
        pidController.setSmartMotionMinOutputVelocity(SmartMotionMinVel,0);
        pidController.setSmartMotionAllowedClosedLoopError(SmartMotionAllowedClosedLoopError, 0);

        shuffleboardTab = Shuffleboard.getTab("Elevator");
        
        ShuffleboardLayout motorList = shuffleboardTab.getLayout("Motor", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
        motorList.addDouble("Motor RPM", () -> encoder.getVelocity());
        motorList.addDouble("Motor Current", () -> motor.getOutputCurrent());
        motorList.addDouble("Motor %", () -> motor.get());
        motorList.addDouble("Motor Position", () -> encoder.getPosition());

        ShuffleboardLayout controlList = shuffleboardTab.getLayout("Control", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
        controlList.addBoolean("Limit Switch", () -> !zeroLimitSwitch.get());
        controlList.addBoolean("Zeroed", () -> (zeroed));
        controlList.addBoolean("At Zero", () -> (atLimitSwitch));
        controlList.addBoolean("At Min", () -> (atMinLimit));
        controlList.addBoolean("At Max", () -> (atMaxLimit));
        controlList.addDouble("Commanded Position", () -> (commandedPosition));

        mainTab.getLayout("Zeroed", BuiltInLayouts.kList).withSize(1, 3).addBoolean("Elevator", () -> zeroed);
    }

    public void teleopInit() {
        pidController.setReference(encoder.getPosition(), commadingControlType);
        // zeroed = false;
    }

    public void checkLimitSwitch() {
        // true - switch is not pressed
        if(zeroLimitSwitch.get()) {
            atLimitSwitch = false;
            return;
        }

        zeroed = true;
        if(!atLimitSwitch) {
            //stop motor and reset encoder position on rising edge
            atLimitSwitch = true;
            encoder.setPosition(0);
            pidController.setReference(0, commadingControlType);
        }
        
    }
    
    public void checkMinLimit() {
        // Position is within lower bound
        if(encoder.getPosition() > ELEVATOR_MIN_POSITION) {
            atMinLimit = false;
            return;
        }

        if(!atMinLimit) {
            //stop motor on rising edge
            atMinLimit = true;
            pidController.setReference(0, commadingControlType);
        }
    }

    public void checkMaxLimit() {
        // Position is within upper bound
        if(encoder.getPosition() < ELEVATOR_MAX_POSITION) {
            atMaxLimit = false;
            return;
        }
                
        if(!atMaxLimit) {
            //stop motor on rising edge
            atMaxLimit = true;
            pidController.setReference(ELEVATOR_MAX_POSITION, commadingControlType);
        }
    }

    /**
     * Sets the controller's reference to a percent of the duty cycle.
     * <p>
     * Will not set a negative percent if atMinLimit is true and will not set a positive percent if atMaxLimit is true.
     * <p>
     * Will not set any position if the mechanism is not zeroed
     * <p>
     * Will set the motor's current limit to the running current limit
     * @param percent the duty cycle percentage (between -1 and 1) to be commanded
     */
    public void setMotorPercent(double percent) {
        //at min and attempting to decrease
        if(atMinLimit && percent < 0 && useLimits)
            return;
        
        //at max or not yet zeroed and attempting to increase
        if((atMaxLimit || !zeroed) && percent > 0 && useLimits)
            return;
        pidController.setReference(percent * (slowMode? ELEVATOR_SLOW_MODE_MULTIPLIER : 1), ControlType.kDutyCycle);
    }

    /**
     * Sets the controller's reference to a position, given the position is between 0 and maxPosition
     * <p>
     * Will set the motor's current limit to the running current limit
     * @param position the mechanism position to be commanded
     */
    public void setPosition(double position) {
        //position out of bounds
        if(position < ELEVATOR_MIN_POSITION || position > ELEVATOR_MAX_POSITION)
            return;
        
        //not zeroed and moving away from limit switch
        if(!zeroed && position > encoder.getPosition())
            return;

        pidController.setReference(position, commadingControlType);
        commandedPosition = position;
    }

    public void gotoSetPosition(int positionId) {
        setPosition(ELEVATOR_POSITIONS_ORDERED[positionId]);
    }

    public Command holdPositionCommand() {
        return this.runOnce(() -> setPosition(encoder.getPosition()));
    }

    public Command lowNodeCommand(BooleanSupplier cubeMode) {
        return this.runOnce(() -> setPosition(cubeMode.getAsBoolean()? ELEVATOR_CUBE_LOW_POSITION: ELEVATOR_CONE_LOW_POSITION));
    }

    public Command midNodeCommand(BooleanSupplier cubeMode) {
        return this.runOnce(() -> setPosition(cubeMode.getAsBoolean()? ELEVATOR_CUBE_MID_POSITION: ELEVATOR_CONE_MID_POSITION));
    }

    public Command highNodeCommand(BooleanSupplier cubeMode) {
        return this.runOnce(() -> setPosition(cubeMode.getAsBoolean()? ELEVATOR_CUBE_HIGH_POSITION: ELEVATOR_CONE_HIGH_POSITION));
    }

    public Command shelfCommand(BooleanSupplier cubeMode) {
        return this.runOnce(() -> setPosition(cubeMode.getAsBoolean()? ELEVATOR_CUBE_SHELF_POSITION: ELEVATOR_CONE_SHELF_POSITION));
    }

    public Command stowCommand() {
        return this.runOnce(() -> setPosition(ELEVATOR_STOW_POSITION));
    }

    public Command groundCommand(BooleanSupplier cubeMode) {
        return this.runOnce(() -> setPosition(cubeMode.getAsBoolean()? ELEVATOR_CUBE_GROUND_POSITION: ELEVATOR_CONE_GROUND_POSITION));
    }

    public Command zeroCommand() {
        return this.runOnce(() -> setPosition(0));
    }

    public Command disableLimitsCommand() {
        return this.runOnce(() -> useLimits = false);
    }

    public Command enableLimitsCommand() {
        return this.runOnce(() -> useLimits = true);
    }

    public Command disableSlowModeCommand() {
        return this.runOnce(() -> slowMode = false);
    }

    public Command enableSlowModeCommand() {
        return this.runOnce(() -> slowMode = true);
    }

    @Override
    public void periodic() {
        checkMinLimit();
        checkMaxLimit();
    }

    public void setPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;

        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
    }

    public double getPosition() {
        return encoder.getPosition();
    }
}
