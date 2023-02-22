package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SingleAxisSubsystem extends SubsystemBase{
    
    protected final CANSparkMax motor;
    protected final DigitalInput zeroLimitSwitch;
    protected final SparkMaxPIDController pidController;
    protected final RelativeEncoder encoder;

    protected final ShuffleboardTab shuffleboardTab;
    protected final GenericEntry motorVelocityEntry;
    protected final GenericEntry motorPercentEntry;
    protected final GenericEntry motorPositionEntry;
    protected final GenericEntry limitSwitchEntry;
    protected final GenericEntry zeroPositionEntry;
    protected final GenericEntry lowNodePositionEntry;
    protected final GenericEntry midNodePositionEntry;
    protected final GenericEntry highNodePositionEntry;
    protected final GenericEntry shelfPositionEntry;
    protected final GenericEntry maxPositionEntry;
    protected final GenericEntry pEntry;
    protected final GenericEntry iEntry;
    protected final GenericEntry dEntry;

    protected double p = 0;
    protected double i = 0;
    protected double d = 0;
    
    protected double zeroPosition = 0;
    protected double lowNodePosition = 0;
    protected double midNodePosition = 0;
    protected double highNodePosition = 0;
    protected double shelfPosition = 0;

    protected double maxPosition = 0;

    public SingleAxisSubsystem(final int MOTOR_ID, final int ZERO_LIMIT_SWITCH_CHANNEL, String shuffleboardTabName) {

        motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
        zeroLimitSwitch = new DigitalInput(ZERO_LIMIT_SWITCH_CHANNEL);
        pidController = motor.getPIDController();
        encoder = motor.getEncoder();
        motor.restoreFactoryDefaults();

        shuffleboardTab = Shuffleboard.getTab(shuffleboardTabName);
        motorVelocityEntry = shuffleboardTab.add("Motor RPM", 0).getEntry();
        motorPercentEntry = shuffleboardTab.add("Motor %", 0).getEntry();
        motorPositionEntry = shuffleboardTab.add("Motor Position", 0).getEntry();
        limitSwitchEntry = shuffleboardTab.add("Limit Switch", false).getEntry();
        zeroPositionEntry = shuffleboardTab.add("Zero Position", zeroPosition).getEntry();
        lowNodePositionEntry = shuffleboardTab.add("Low Node Position", lowNodePosition).getEntry();
        midNodePositionEntry = shuffleboardTab.add("Mid Node Position", midNodePosition).getEntry();
        highNodePositionEntry = shuffleboardTab.add("High Node Position", highNodePosition).getEntry();
        shelfPositionEntry = shuffleboardTab.add("Shelf Position", shelfPosition).getEntry();
        maxPositionEntry = shuffleboardTab.add("Max Position", maxPosition).getEntry();
        pEntry = shuffleboardTab.add("P Gain", p).getEntry();
        iEntry = shuffleboardTab.add("I Gain", i).getEntry();
        dEntry = shuffleboardTab.add("D Gain", d).getEntry();
    }

    public boolean atZeroLimitSwitch() {
        if(zeroLimitSwitch.get())
            return false;
        zeroPosition = encoder.getPosition();
        pidController.setReference(0, ControlType.kDutyCycle);
        return true;
    }

    public boolean atMaxLimit() {
        if(encoder.getPosition() + zeroPosition < maxPosition)
            return false;
        pidController.setReference(0, ControlType.kDutyCycle);
        return true;
    }

    public void setMotorPercent(double percent) {
        if(atZeroLimitSwitch() || atMaxLimit())
            return;
        pidController.setReference(percent, ControlType.kDutyCycle);
    }

    public void setPosition(double position) {
        if(atZeroLimitSwitch() || atMaxLimit())
            return;
        pidController.setReference(position + zeroPosition, ControlType.kPosition);
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

    protected void setPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;

        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
    }

    @Override
    public void periodic() {
        //Update shuffleboard values
        motorVelocityEntry.setDouble(encoder.getVelocity());
        motorPercentEntry.setDouble(motor.get());
        motorPositionEntry.setDouble(encoder.getPosition());
        limitSwitchEntry.setBoolean(zeroLimitSwitch.get());
        zeroPositionEntry.setDouble(zeroPosition);
        
        //Fetch values from shuffleboard
        lowNodePosition = lowNodePositionEntry.getDouble(lowNodePosition);
        midNodePosition = midNodePositionEntry.getDouble(midNodePosition);
        highNodePosition = highNodePositionEntry.getDouble(highNodePosition);
        shelfPosition = shelfPositionEntry.getDouble(shelfPosition);

        double newP = pEntry.getDouble(p);
        double newI = iEntry.getDouble(i);
        double newD = dEntry.getDouble(d);

        if(newP != p) {
            pidController.setP(newP);
            p = newP;
        }
        
        if(newI != i) {
            pidController.setI(newI);
            i = newI;
        }

        if(newD != d) {
            pidController.setD(newD);
            d = newD;
        }

        atZeroLimitSwitch();
        atMaxLimit();
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
