package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.*;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class ClawSubsystem extends SubsystemBase{
    private Compressor compressor = new Compressor(2, PneumaticsModuleType.REVPH);
    private DoubleSolenoid solenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, CLAW_SOLENOID_FORWARD_CHANNEL, CLAW_SOLENOID_REVERSE_CHANNEL);
    private TimeOfFlight tofSensor = new TimeOfFlight(TOF_SENSOR_ID);

    private ShuffleboardTab tab = Shuffleboard.getTab("Claw");

    private boolean open = false;

    private double closeRange = 10;
    private boolean autoClose = false;
    private Trigger gamePieceInRangeTrigger;

    public ClawSubsystem() {
        compressor.enableDigital();
        tofSensor.setRangingMode(RangingMode.Short, 0.05D);

        // Auto close claw when game piece is in range
        gamePieceInRangeTrigger = new Trigger(() -> (getDistanceToGamePiece() <= closeRange));
        gamePieceInRangeTrigger.and(() -> autoClose).onTrue(closeCommand());

        tab.addBoolean("Open", () -> open);
        tab.addDouble("Game Piece Distance", () -> tofSensor.getRange());
    }

    public Command disableCompressorCommand() {
        return this.run(() -> compressor.disable());
    }

    public Command enableCompressorCommand() {
        return this.run(() -> compressor.enableDigital());
    }

    public Command openCommand() {
        open = true;
        return this.runOnce(() -> solenoid.set(kReverse));
    }

    public Command closeCommand() {
        open = false;
        return this.runOnce(() -> solenoid.set(kForward));
    }

    public Command toggleCommand() {
        if(open) {
            return closeCommand();
        }
        
        return openCommand();
    }

    /**
     * 
     * @return the distance to game piece (mm) or -1 if the sensor does not detect a game piece
     */
    public double getDistanceToGamePiece() {
        if(!tofSensor.isRangeValid())
            return -1;
        
        return tofSensor.getRange();
    }

    public TimeOfFlight getTOF() {
        return tofSensor;
    }

}
