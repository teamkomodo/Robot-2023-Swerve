package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;

import static frc.robot.Constants.*;

public class IntakePieceCommand extends CommandBase{

    private final IntakeSubsystem intakeSubsystem;
    private final JointSubsystem jointSubsystem;
    private final LEDStripSubsystem ledStripSubsystem;

    private boolean holdingPiece;
    private long startTime;
    private long clampStart;

    public IntakePieceCommand(IntakeSubsystem intakeSubsystem, JointSubsystem jointSubsystem, LEDStripSubsystem ledStripSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.jointSubsystem = jointSubsystem;
        this.ledStripSubsystem = ledStripSubsystem;

        addRequirements(intakeSubsystem, jointSubsystem, ledStripSubsystem);
    }

    @Override
    public void initialize() {
        holdingPiece = false;
        startTime = RobotController.getFPGATime();
        clampStart = -1;
    }

    @Override
    public void end(boolean interrupted) {
        if(!holdingPiece)
            intakeSubsystem.setMotorDutyCycle(0);
    }

    @Override
    public void execute() {
        if(holdingPiece) {
            ledStripSubsystem.setPattern(0.75); // Solid Color Dark Green
            return;
        }

        ledStripSubsystem.setPattern(0.19); // Color 2 Larson Scanner
        intakeSubsystem.setMotorVelocity(-4000);

        if(intakeSubsystem.pieceDetected()) {
            if(clampStart == -1)
                clampStart = RobotController.getFPGATime();
            jointSubsystem.setPosition(jointSubsystem.getPosition() + 5);
        }

        if(Math.abs(intakeSubsystem.getSmoothCurrent()) > INTAKE_THRESHOLD_CURRENT && RobotController.getFPGATime() - startTime > 500000 && RobotController.getFPGATime() - clampStart > 500000) {
            intakeSubsystem.setMotorDutyCycle(-0.2);
            jointSubsystem.setPosition(JOINT_STOW_POSITION);
            holdingPiece = true;
        }
    }
    
}
