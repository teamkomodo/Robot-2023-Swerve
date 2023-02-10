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

public class ElevatorSubsystem extends SubsystemBase{

    private final CANSparkMax elevatorMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final DigitalInput zeroLimitSwitch = new DigitalInput(1);
    private final SparkMaxPIDController pidController = elevatorMotor.getPIDController();
    private final RelativeEncoder encoder = elevatorMotor.getEncoder();
    
    private double zeroPosition = 0;
    private double lowNodePosition = 0;
    private double midNodePosition = 0;
    private double highNodePosition = 0;
    private double shelfPosition = 0;

    //Shuffleboard Stuff
    private final ShuffleboardTab elevTab = Shuffleboard.getTab("Elevator");
    private final GenericEntry motorVelocityEntry = elevTab.add("Motor RPM", 0).getEntry();
    private final GenericEntry motorSpeedEntry = elevTab.add("Motor %", 0).getEntry();
    private final GenericEntry motorPositionEntry = elevTab.add("Motor Position", 0).getEntry();
    private final GenericEntry limitSwitchEntry = elevTab.add("Limit Switch", false).getEntry();
    private final GenericEntry zeroPositionEntry = elevTab.add("Zero Position", zeroPosition).getEntry();
    private final GenericEntry lowNodePositionEntry = elevTab.add("Low Node Position", lowNodePosition).getEntry();
    private final GenericEntry midNodePositionEntry = elevTab.add("Mid Node Position", midNodePosition).getEntry();
    private final GenericEntry highNodePositionEntry = elevTab.add("High Node Position", highNodePosition).getEntry();
    private final GenericEntry shelfPositionEntry = elevTab.add("Shelf Position", shelfPosition).getEntry();

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
        motorVelocityEntry.setDouble(elevatorMotor.getEncoder().getVelocity());
        motorSpeedEntry.setDouble(elevatorMotor.get());
        motorPositionEntry.setDouble(elevatorMotor.getEncoder().getPosition());
        limitSwitchEntry.setBoolean(zeroLimitSwitch.get());
        
        //Fetch values from shuffleboard
        zeroPosition = zeroPositionEntry.getDouble(zeroPosition);
        lowNodePosition = lowNodePositionEntry.getDouble(lowNodePosition);
        midNodePosition = midNodePositionEntry.getDouble(midNodePosition);
        highNodePosition = highNodePositionEntry.getDouble(highNodePosition);
        shelfPosition = shelfPositionEntry.getDouble(shelfPosition);

        if(zeroLimitSwitch.get()) {
            zeroPosition = encoder.getPosition();
            setElevatorPercent(0);
        }
    }
}
