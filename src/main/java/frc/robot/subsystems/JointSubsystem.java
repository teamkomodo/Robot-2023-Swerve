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

public class JointSubsystem extends SubsystemBase{
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
    private final GenericEntry minPositionEntry;
    private final GenericEntry atZeroEntry;
    private final GenericEntry atMaxEntry;
    private final GenericEntry commandedPositionEntry;

    private double p = 1.0;
    private double i = 1.0e-6;
    private double d = 1.0;
    
    private double lowNodePosition = 0;
    private double midNodePosition = 0;
    private double highNodePosition = 0;
    private double shelfPosition = 0;

    private double minPosition = 0;
    private double maxPosition = 0;

    private boolean atMaxLimit = false;
    private boolean atMinLimit = false;

    public JointSubsystem() {

        motor = new CANSparkMax(JOINT_MOTOR_ID, MotorType.kBrushless);
        zeroLimitSwitch = new DigitalInput(JOINT_ZERO_SWITCH_CHANNEL);
        motor.restoreFactoryDefaults();
        pidController = motor.getPIDController();
        encoder = motor.getEncoder();
        motor.setInverted(false);
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);

        shuffleboardTab = Shuffleboard.getTab("Joint");
        ShuffleboardLayout positionList = shuffleboardTab.getLayout("Positions", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
        ShuffleboardLayout motorList = shuffleboardTab.getLayout("Motor", BuiltInLayouts.kList).withSize(2, 2).withPosition(2, 0);
        motorVelocityEntry = motorList.add("Motor RPM", 0).getEntry();
        motorPercentEntry = motorList.add("Motor %", 0).getEntry();
        motorPositionEntry = motorList.add("Motor Position", 0).getEntry();
        atZeroEntry = shuffleboardTab.add("At Zero", false).getEntry();
        atMaxEntry = shuffleboardTab.add("At Max", false).getEntry();
        limitSwitchEntry = shuffleboardTab.add("Limit Switch", false).withPosition(4, 0).getEntry();
        lowNodePositionEntry = positionList.add("Low Node Position", lowNodePosition).getEntry();
        midNodePositionEntry = positionList.add("Mid Node Position", midNodePosition).getEntry();
        highNodePositionEntry = positionList.add("High Node Position", highNodePosition).getEntry();
        shelfPositionEntry = positionList.add("Shelf Position", shelfPosition).getEntry();
        maxPositionEntry = positionList.add("Max Position", maxPosition).getEntry();
        minPositionEntry = positionList.add("Min Position", minPosition).getEntry();
        commandedPositionEntry = shuffleboardTab.add("Commanded Position", 0).getEntry();
    }
    
    public boolean checkMinLimit() {
        if(encoder.getPosition() > minPosition)
            return false;
        
        //false - switch is active
        if(!atMinLimit) {
            //stop motor on rising edge
            pidController.setReference(0, ControlType.kDutyCycle);
        }
        return true;
    }

    public boolean checkMaxLimit() {
        if(encoder.getPosition() < maxPosition)
            return false;
        
        if(!atMaxLimit) {
            //stop motor on rising edge
            pidController.setReference(0, ControlType.kDutyCycle);
        }
        return true;
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
        pidController.setReference(percent, ControlType.kDutyCycle);
    }

    /**
     * Sets the controller's reference to a position, given the position is between minPosition and maxPosition
     * @param position the mechanism position to be commanded
     */
    public void setPosition(double position) {
        if(position < minPosition || position > maxPosition)
            return;

        pidController.setReference(position, ControlType.kPosition);
        commandedPositionEntry.setDouble(position);
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
        atZeroEntry.setBoolean(atMinLimit);
        atMaxEntry.setBoolean(atMaxLimit);
        limitSwitchEntry.setBoolean(zeroLimitSwitch.get());
        
        //Fetch values from shuffleboard
        lowNodePosition = lowNodePositionEntry.getDouble(lowNodePosition);
        midNodePosition = midNodePositionEntry.getDouble(midNodePosition);
        highNodePosition = highNodePositionEntry.getDouble(highNodePosition);
        shelfPosition = shelfPositionEntry.getDouble(shelfPosition);
        maxPosition = maxPositionEntry.getDouble(maxPosition);
        minPosition = minPositionEntry.getDouble(minPosition);

        atMinLimit = checkMinLimit();
        atMaxLimit = checkMaxLimit();
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

    public void setMinPosition(double minPosition) {
        this.minPosition = minPosition;
        minPositionEntry.setDouble(minPosition);
    }
}
