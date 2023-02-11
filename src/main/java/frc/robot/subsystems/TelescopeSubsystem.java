package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class TelescopeSubsystem extends SubsystemBase{

    private final CANSparkMax telescopeMotor = new CANSparkMax(TELESCOPE_MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput zeroLimitSwitch = new DigitalInput(TELESCOPE_ZERO_SWITCH_CHANNEL);
    private final SparkMaxPIDController pidController = telescopeMotor.getPIDController();
    private final RelativeEncoder encoder = telescopeMotor.getEncoder();

    private double zeroPosition = 0.0;
    private double lowNodePosition = 0.0;
    private double midNodePosition = 0.0;
    private double highNodePosition = 0.0;
    private double shelfPosition = 0.0;

    private final ShuffleboardTab telescopeTab = Shuffleboard.getTab("Telescope");
    private final GenericEntry motorVelocityEntry = telescopeTab.add("Motor RPM", 0).getEntry();
    private final GenericEntry motorPercentEntry = telescopeTab.add("Motor %", 0).getEntry();
    private final GenericEntry motorPositionEntry = telescopeTab.add("Motor Position", 0).getEntry();
    private final GenericEntry limitSwitchEntry = telescopeTab.add("Limit Switch", false).getEntry();
    private final GenericEntry zeroPositionEntry = telescopeTab.add("Zero Position", zeroPosition).getEntry();
    private final GenericEntry lowNodePositionEntry = telescopeTab.add("Low Node Position", lowNodePosition).getEntry();
    private final GenericEntry midNodePositionEntry = telescopeTab.add("Mid Node Position", midNodePosition).getEntry();
    private final GenericEntry highNodePositionEntry = telescopeTab.add("High Node Position", highNodePosition).getEntry();
    private final GenericEntry shelfPositionEntry = telescopeTab.add("Shelf Position", shelfPosition).getEntry();

    public void setTelescopePercent(double percent) {
        pidController.setReference(percent, ControlType.kDutyCycle);
    }

    public void setTelescopePosition(double position) {
        pidController.setReference(position + zeroPosition, ControlType.kPosition);
    }

    public Command runLowNodeCommand() {
        return this.runOnce(() -> setTelescopePosition(lowNodePosition));
    }

    public Command runMidNodeCommand() {
        return this.runOnce(() -> setTelescopePosition(midNodePosition));
    }

    public Command runHighNodeCommand() {
        return this.runOnce(() -> setTelescopePosition(highNodePosition));
    }

    public Command runShelfCommand() {
        return this.runOnce(() -> setTelescopePosition(shelfPosition));
    }

    public Command runZeroCommand() {
        return this.runOnce(() -> setTelescopePosition(0));
    }

    @Override
    public void periodic() {
        //Update shuffleboard values
        motorVelocityEntry.setDouble(encoder.getVelocity());
        motorPercentEntry.setDouble(telescopeMotor.get());
        motorPositionEntry.setDouble(encoder.getPosition());
        limitSwitchEntry.setBoolean(zeroLimitSwitch.get());
        
        //Fetch values from shuffleboard
        zeroPosition = zeroPositionEntry.getDouble(zeroPosition);
        lowNodePosition = lowNodePositionEntry.getDouble(lowNodePosition);
        midNodePosition = midNodePositionEntry.getDouble(midNodePosition);
        highNodePosition = highNodePositionEntry.getDouble(highNodePosition);
        shelfPosition = shelfPositionEntry.getDouble(shelfPosition);
        if(!zeroLimitSwitch.get()) {
            setTelescopePercent(0);
            zeroPosition = encoder.getPosition();
        }
    }
    
}
