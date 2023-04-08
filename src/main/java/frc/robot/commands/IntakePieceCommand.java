package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.*;

public class IntakePieceCommand extends CommandBase{

    private final IntakeSubsystem intakeSubsystem;

    private boolean atSpeed = false;

    public IntakePieceCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        atSpeed = false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotorDutyCycle(0);
    }

    @Override
    public void execute() {
        intakeSubsystem.setMotorDutyCycle(-0.2);
        if(Math.abs(intakeSubsystem.getEncoder().getVelocity()) > INTAKE_THRESHOLD_VELOCITY) {
            atSpeed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(intakeSubsystem.getEncoder().getVelocity()) < INTAKE_THRESHOLD_VELOCITY && atSpeed;
    }
    
}
