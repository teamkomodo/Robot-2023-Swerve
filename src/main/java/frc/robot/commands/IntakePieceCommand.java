package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;

import static frc.robot.Constants.*;

public class IntakePieceCommand extends CommandBase{

    private final IntakeSubsystem intakeSubsystem;
    private final LEDStripSubsystem ledStripSubsystem;

    private boolean atSpeed = false;

    private boolean finished = false;

    public IntakePieceCommand(IntakeSubsystem intakeSubsystem, LEDStripSubsystem ledStripSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.ledStripSubsystem = ledStripSubsystem;

        addRequirements(intakeSubsystem, ledStripSubsystem);
    }

    @Override
    public void initialize() {
        atSpeed = false;
    }

    @Override
    public void execute() {
        if(finished) {
            ledStripSubsystem.setPattern(0.75); // Solid Color Dark Green
            return;
        }

        ledStripSubsystem.setPattern(0.19); // Color 2 Larson Scanner
        intakeSubsystem.setMotorDutyCycle(-0.2);
        
        if(Math.abs(intakeSubsystem.getEncoder().getVelocity()) > INTAKE_THRESHOLD_VELOCITY) {
            atSpeed = true;
        }

        if(Math.abs(intakeSubsystem.getEncoder().getVelocity()) < INTAKE_THRESHOLD_VELOCITY && atSpeed) {
            intakeSubsystem.setMotorDutyCycle(0);
            finished = true;
        }
    }
    
}
