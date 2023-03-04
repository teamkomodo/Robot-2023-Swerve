package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ElevatorSubsystem extends SubsystemBase{
    
    // 9:1 reduction
    // 5.53 in circumference
    private final double INCHES_PER_REVOLUTION = 3.53D/9.0D;

    private final CANSparkMax motor;
    private final DigitalInput zeroLimitSwitch;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder encoder;

    private final ShuffleboardTab shuffleboardTab;
    private final GenericEntry motorVelocityEntry;
    private final GenericEntry motorPercentEntry;
    private final GenericEntry motorPositionEntry;
    private final GenericEntry limitSwitchEntry;
    private final GenericEntry lowNodePositionEntry;
    private final GenericEntry midNodePositionEntry;
    private final GenericEntry highNodePositionEntry;
    private final GenericEntry shelfPositionEntry;
    private final GenericEntry maxPositionEntry;
    private final GenericEntry currentLimitEntry;

    private double p = 1.0;
    private double i = 1.0e-6;
    private double d = 0.7;
    
    private double lowNodePosition = 12;
    private double midNodePosition = 16;
    private double highNodePosition = 20;
    private double shelfPosition = 25;

    private double maxPosition = 36;

    private boolean atMaxLimit = false;
    private boolean atMinLimit = false;

    private int holdingCurrentLimit = 30;
    private int runningCurrentLimit = 60;

    private double commandedPosition = 0;

    private long lastTimeMillis = 0;
    private long holdingTimeMillis = 0;
    private long minHoldingTime = 1000;

    private double holdingVelocityThreshold = 1;

    public ElevatorSubsystem() {

        zeroLimitSwitch = new DigitalInput(ELEVATOR_ZERO_SWITCH_CHANNEL);

        motor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setSmartCurrentLimit(holdingCurrentLimit, runningCurrentLimit);

        pidController = motor.getPIDController();
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(INCHES_PER_REVOLUTION);
        
        shuffleboardTab = Shuffleboard.getTab("Elevator");
        ShuffleboardLayout positionList = shuffleboardTab.getLayout("Positions", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
        ShuffleboardLayout motorList = shuffleboardTab.getLayout("Motor", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
        motorVelocityEntry = motorList.add("Motor RPM", 0).getEntry();
        motorPercentEntry = motorList.add("Motor %", 0).getEntry();
        motorPositionEntry = motorList.add("Motor Position", 0).getEntry();
        limitSwitchEntry = shuffleboardTab.add("Limit Switch", false).withPosition(6, 0).getEntry();
        lowNodePositionEntry = positionList.add("Low Node Position", lowNodePosition).getEntry();
        midNodePositionEntry = positionList.add("Mid Node Position", midNodePosition).getEntry();
        highNodePositionEntry = positionList.add("High Node Position", highNodePosition).getEntry();
        shelfPositionEntry = positionList.add("Shelf Position", shelfPosition).getEntry();
        maxPositionEntry = positionList.add("Max Position", maxPosition).getEntry();
        currentLimitEntry = shuffleboardTab.add("Current Limit", runningCurrentLimit).getEntry();

        shuffleboardTab.addBoolean("At Zero", () -> (atMinLimit));
        shuffleboardTab.addBoolean("At Max", () -> (atMaxLimit));
        shuffleboardTab.addDouble("Commanded Position", () -> (commandedPosition));

    }
    
    public void checkMinLimit() {
        //true - switch is not active
        if(zeroLimitSwitch.get()) {
            atMinLimit = false;
            return;
        }

        //false - switch is active
        if(!atMinLimit) {
            //stop motor and reset encoder position on rising edge
            atMinLimit = true;
            useHoldingCurrentLimit();
            encoder.setPosition(0);
            pidController.setReference(0, ControlType.kPosition);
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
            useHoldingCurrentLimit();
            pidController.setReference(maxPosition, ControlType.kPosition);
        }
    }

    /**
     * <p>Sets the controller's reference to a percent of the duty cycle.</p>
     * Will not set a negative percent if atMinLimit is true and will not set a positive percent if atMaxLimit is true.
     * @param percent the duty cycle percentage (between -1 and 1) to be commanded
     */
    public void setMotorPercent(double percent) {
        //at min and attempting to decrease
        if(atMinLimit && percent < 0)
            return;
        
        //at max and attempting to increase
        if(atMaxLimit && percent > 0)
            return;
        useRunningCurrentLimit();
        pidController.setReference(percent, ControlType.kDutyCycle);
    }

    /**
     * Sets the controller's reference to a position, given the position is between 0 and maxPosition
     * @param position the mechanism position to be commanded
     */
    public void setPosition(double position) {
        if(position < 0 || position > maxPosition)
            return;

        useRunningCurrentLimit();
        pidController.setReference(position, ControlType.kPosition);
        commandedPosition = position;
    }

    public Command runHoldPositionCommand() {
        return this.runOnce(() -> setPosition(encoder.getPosition()));
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

    public Command runZeroCommand() {
        return this.runOnce(() -> setPosition(0));
    }

    @Override
    public void periodic() {
        //Update shuffleboard values
        motorVelocityEntry.setDouble(encoder.getVelocity());
        motorPercentEntry.setDouble(motor.get());
        motorPositionEntry.setDouble(encoder.getPosition());
        limitSwitchEntry.setBoolean(zeroLimitSwitch.get());

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
    }

    public void setPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;

        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
    }

    public void useRunningCurrentLimit() {
        currentLimitEntry.setInteger(runningCurrentLimit);
        //motor.setSmartCurrentLimit(runningCurrentLimit);
    }

    public void useHoldingCurrentLimit() {
        currentLimitEntry.setInteger(holdingCurrentLimit);
        //motor.setSmartCurrentLimit(holdingCurrentLimit);
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
