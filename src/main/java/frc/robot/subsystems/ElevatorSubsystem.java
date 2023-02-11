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

import static frc.robot.Constants.*;

public class ElevatorSubsystem extends SubsystemBase{

    private final CANSparkMax motor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput zeroLimitSwitch = new DigitalInput(ELEVATOR_ZERO_SWITCH_CHANNEL);
    private final SparkMaxPIDController pidController = motor.getPIDController();
    private final RelativeEncoder encoder = motor.getEncoder();

    private double p = 1e-1;
    private double i = 1e-6;
    private double d = 1;
    
    private double zeroPosition = 0;
    private double lowNodePosition = 0;
    private double midNodePosition = 0;
    private double highNodePosition = 0;
    private double shelfPosition = 0;

    //Shuffleboard Stuff
    private final ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
    private final GenericEntry motorVelocityEntry = elevatorTab.add("Motor RPM", 0).getEntry();
    private final GenericEntry motorPercentEntry = elevatorTab.add("Motor %", 0).getEntry();
    private final GenericEntry motorPositionEntry = elevatorTab.add("Motor Position", 0).getEntry();
    private final GenericEntry limitSwitchEntry = elevatorTab.add("Limit Switch", false).getEntry();
    private final GenericEntry zeroPositionEntry = elevatorTab.add("Zero Position", zeroPosition).getEntry();
    private final GenericEntry lowNodePositionEntry = elevatorTab.add("Low Node Position", lowNodePosition).getEntry();
    private final GenericEntry midNodePositionEntry = elevatorTab.add("Mid Node Position", midNodePosition).getEntry();
    private final GenericEntry highNodePositionEntry = elevatorTab.add("High Node Position", highNodePosition).getEntry();
    private final GenericEntry shelfPositionEntry = elevatorTab.add("Shelf Position", shelfPosition).getEntry();
    private final GenericEntry pEntry = elevatorTab.add("P Gain", p).getEntry();
    private final GenericEntry iEntry = elevatorTab.add("I Gain", i).getEntry();
    private final GenericEntry dEntry = elevatorTab.add("D Gain", d).getEntry();

    public ElevatorSubsystem() {
        motor.restoreFactoryDefaults();
    }

    public void setElevatorPercent(double percent) {
        pidController.setReference(percent, ControlType.kDutyCycle);
    }

    public void setElevatorPosition(double position) {
        pidController.setReference(position + zeroPosition, ControlType.kPosition);
    }

    public Command runLowNodeCommand() {
        return this.runOnce(() -> setElevatorPosition(lowNodePosition));
    }

    public Command runMidNodeCommand() {
        return this.runOnce(() -> setElevatorPosition(midNodePosition));
    }

    public Command runHighNodeCommand() {
        return this.runOnce(() -> setElevatorPosition(highNodePosition));
    }

    public Command runShelfCommand() {
        return this.runOnce(() -> setElevatorPosition(shelfPosition));
    }

    public Command runZeroCommand() {
        return this.runOnce(() -> setElevatorPosition(0));
    }

    @Override
    public void periodic() {
        //Update shuffleboard values
        motorVelocityEntry.setDouble(encoder.getVelocity());
        motorPercentEntry.setDouble(motor.get());
        motorPositionEntry.setDouble(encoder.getPosition());
        limitSwitchEntry.setBoolean(zeroLimitSwitch.get());
        
        //Fetch values from shuffleboard
        zeroPosition = zeroPositionEntry.getDouble(zeroPosition);
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

        if(!zeroLimitSwitch.get()) {
            zeroPosition = encoder.getPosition();
            setElevatorPercent(0);
        }
    }
}
