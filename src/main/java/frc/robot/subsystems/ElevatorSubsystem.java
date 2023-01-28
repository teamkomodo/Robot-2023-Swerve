package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{

    private final CANSparkMax elevatorMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final DigitalInput topLimitSwitch = new DigitalInput(1);
    private final ShuffleboardTab elevTab = Shuffleboard.getTab("Elevator");
    private GenericEntry elevatorTopPosition = elevTab.add("Top Position", 0).getEntry();
    private GenericEntry elevatorMotorSlowSpeedMultiplier = elevTab.add("Approach Speed", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .getEntry();

    private final GenericEntry motorVelocityEntry = elevTab.add("Motor Velocity", 0).getEntry();
    private final GenericEntry motorSpeedEntry = elevTab.add("Motor Speed", 0).getEntry();
    private final GenericEntry motorPositionEntry = elevTab.add("Motor Position", 0).getEntry();


    public void setElevatorSpeed(double speed) {
        System.out.println("setElevatorSpeed");
        if(topLimitSwitch.get()){
            elevatorMotor.set(0);
            return;
        }

        if(elevatorMotor.getEncoder().getPosition() < elevatorTopPosition.getDouble(0)) {
            elevatorMotor.set(speed);
        }else {
            elevatorMotor.set(speed * elevatorMotorSlowSpeedMultiplier.getDouble(0));
        }
    }

    @Override
    public void periodic() {
        motorVelocityEntry.setDouble(elevatorMotor.getEncoder().getVelocity());
        motorSpeedEntry.setDouble(elevatorMotor.get());
        motorPositionEntry.setDouble(elevatorMotor.getEncoder().getPosition());
    }
}
