package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class JointSubsystem extends SubsystemBase{
    
    private final CANSparkMax motor;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder encoder;
    private final SparkMaxLimitSwitch reverseSwitch;

    private final ShuffleboardTab shuffleboardTab;
    private final GenericEntry lowNodePositionEntry;
    private final GenericEntry midNodePositionEntry;
    private final GenericEntry highNodePositionEntry;
    private final GenericEntry shelfPositionEntry;
    private final GenericEntry maxPositionEntry;

    private double p = 1.0;
    private double i = 1.0e-6;
    private double d = 0.7;
    
    private double stowPosition = 5;
    private double groundPosition = 6;
    private double lowNodePosition = 7;
    private double midNodePosition = 8;
    private double highNodePosition = 9;
    private double shelfPosition = 10;

    //Order of array corresponds to selector number
    private double[] positions = {stowPosition, groundPosition, shelfPosition, lowNodePosition, midNodePosition, highNodePosition};

    private double minPosition = 5;
    private double maxPosition = 10;

    private boolean atLimitSwitch = false;
    private boolean atMaxLimit = false;
    private boolean atMinLimit = false;

    private int holdingCurrentLimit = 30;
    private int runningCurrentLimit = 60;

    private double commandedPosition = 0;

    private boolean useLimits = true;
    private boolean slowMode = false;

    private boolean zeroed = false;

    public JointSubsystem(ShuffleboardTab mainTab) {

        motor = new CANSparkMax(JOINT_MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setSmartCurrentLimit(holdingCurrentLimit, runningCurrentLimit);

        reverseSwitch = motor.getReverseLimitSwitch(Type.kNormallyOpen);

        encoder = motor.getEncoder();
        encoder.setPosition(0);

        pidController = motor.getPIDController();
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setReference(encoder.getPosition(), ControlType.kPosition);
        
        shuffleboardTab = Shuffleboard.getTab("Joint");
        ShuffleboardLayout positionList = shuffleboardTab.getLayout("Positions", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
        ShuffleboardLayout motorList = shuffleboardTab.getLayout("Motor", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
        lowNodePositionEntry = positionList.add("Low Node Position", lowNodePosition).getEntry();
        midNodePositionEntry = positionList.add("Mid Node Position", midNodePosition).getEntry();
        highNodePositionEntry = positionList.add("High Node Position", highNodePosition).getEntry();
        shelfPositionEntry = positionList.add("Shelf Position", shelfPosition).getEntry();
        maxPositionEntry = positionList.add("Max Position", maxPosition).getEntry();

        motorList.addDouble("Motor RPM", () -> encoder.getVelocity());
        motorList.addDouble("Motor Current", () -> motor.getOutputCurrent());
        motorList.addDouble("Motor %", () -> motor.get());
        motorList.addDouble("Motor Position", () -> encoder.getPosition());

        shuffleboardTab.addBoolean("Limit Switch", () -> reverseSwitch.isPressed());
        shuffleboardTab.addBoolean("Zeroed", () -> (zeroed));
        shuffleboardTab.addBoolean("At Min", () -> (atMinLimit));
        shuffleboardTab.addBoolean("At Max", () -> (atMaxLimit));
        shuffleboardTab.addDouble("Commanded Position", () -> (commandedPosition));

        mainTab.getLayout("Zeroed", BuiltInLayouts.kList).addBoolean("Joint", () -> zeroed);

    }

    public void checkLimitSwitch() {
        if(reverseSwitch.isPressed()) {
            atLimitSwitch = false;
            return;
        }
        
        if(!atLimitSwitch) {
            //stop motor and reset encoder position on rising edge
            atLimitSwitch = true;
            zeroed = true;
            encoder.setPosition(0);
            pidController.setReference(0, ControlType.kPosition);
        }
        
    }
    
    public void checkMinLimit() {
        if(encoder.getPosition() > minPosition) {
            atMinLimit = false;
            return;
        }

        if(!atMinLimit) {
            atMinLimit = true;
            pidController.setReference(minPosition, ControlType.kPosition);
        }
    }

    public void checkMaxLimit() {
        if(encoder.getPosition() < maxPosition) {
            atMaxLimit = false;
            return;
        }
                
        if(!atMaxLimit) {
            //stop motor on rising edge
            atMaxLimit = true;
            pidController.setReference(maxPosition, ControlType.kPosition);
        }
    }

    /**
     * Sets the controller's reference to a percent of the duty cycle.
     * <p>
     * Will not set a negative percent if atMinLimit is true and will not set a positive percent if atMaxLimit is true.
     * <p>
     * Will set the motor's current limit to the running current limit
     * @param percent the duty cycle percentage (between -1 and 1) to be commanded
     */
    public void setMotorPercent(double percent) {
        //at min and attempting to decrease and zeroed (allow movement past limit if not yet zeroed)
        if(atMinLimit && percent < 0 && zeroed && useLimits)
            return;
        
        //at max or not yet zeroed and attempting to increase
        if((atMaxLimit || !zeroed) && percent > 0 && useLimits)
            return;
        pidController.setReference(percent * (slowMode? JOINT_SLOWLMODE_MULTIPLIER : 1), ControlType.kDutyCycle);
    }

    /**
     * Sets the controller's reference to a position, given the position is between 0 and maxPosition
     * <p>
     * Will set the motor's current limit to the running current limit
     * @param position the mechanism position to be commanded
     */
    public void setPosition(double position) {
        //position out of bounds
        if(position < minPosition || position > maxPosition)
            return;
        
        //not zeroed and moving away from limit switch
        if(!zeroed & position > encoder.getPosition())
            return;

        pidController.setReference(position, ControlType.kPosition);
        commandedPosition = position;
    }

    public void gotoSetPosition(int positionId) {
        setPosition(positions[positionId]);
    }

    public Command runZeroCommand() {
        return this.runOnce(() -> encoder.setPosition(0));
    }

    public Command runHoldPositionCommand() {
        return this.runOnce(() -> pidController.setReference(encoder.getPosition(), ControlType.kPosition));
    }

    public Command runLowNodeCommand() {
        return this.runOnce(() -> setPosition(lowNodePosition));
    }

    public Command runMidNodeCommand() {
        return this.runOnce(() -> setPosition(midNodePosition));
    }

    public Command runHighNodeCommand() {
        return this.runOnce(() -> setPosition(highNodePosition));
    }

    public Command runShelfCommand() {
        return this.runOnce(() -> setPosition(shelfPosition));
    }

    public Command runDisableLimitsCommand() {
        return this.runOnce(() -> useLimits = false);
    }

    public Command runEnableLimitsCommand() {
        return this.runOnce(() -> useLimits = true);
    }

    public Command runDisableSlowModeCommand() {
        return this.runOnce(() -> slowMode = false);
    }

    public Command runEnableSlowModeCommand() {
        return this.runOnce(() -> slowMode = true);
    }

    @Override
    public void periodic() {

        //Fetch values from shuffleboard
        lowNodePosition = lowNodePositionEntry.getDouble(lowNodePosition);
        midNodePosition = midNodePositionEntry.getDouble(midNodePosition);
        highNodePosition = highNodePositionEntry.getDouble(highNodePosition);
        shelfPosition = shelfPositionEntry.getDouble(shelfPosition);

        // long currentTimeMillis = System.currentTimeMillis();
        // if(encoder.getVelocity() < holdingVelocityThreshold) {
        //     holdingTimeMillis += currentTimeMillis - lastTimeMillis;
        // }else {
        //     holdingTimeMillis = 0;
        // }

        // if(holdingTimeMillis > minHoldingTime) {
        //     useHoldingCurrentLimit();
        // }else {
        //     useRunningCurrentLimit();
        // }
        
        // lastTimeMillis = System.currentTimeMillis();

        checkMinLimit();
        checkMaxLimit();
        checkLimitSwitch();
    }

    public void setPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;

        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
    }

    public void setLowNodePosition(double lowNodePosition) {
        this.lowNodePosition = lowNodePosition;
        lowNodePositionEntry.setDouble(lowNodePosition);
    }

    public void setMidNodePosition(double midNodePosition) {
        this.midNodePosition = lowNodePosition;
        midNodePositionEntry.setDouble(midNodePosition);
    }

    public void setHighNodePosition(double highNodePosition) {
        this.highNodePosition = lowNodePosition;
        highNodePositionEntry.setDouble(highNodePosition); 
    }

    public void setShelfPosition(double shelfPosition) {
        this.shelfPosition = shelfPosition;
        shelfPositionEntry.setDouble(shelfPosition);
    }

    public void setMaxPosition(double maxPosition) {
        this.maxPosition = maxPosition;
        maxPositionEntry.setDouble(maxPosition);
    }
}
