package frc.robot.auto.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.util.AutoCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JointSubsystem;

import static frc.robot.Constants.*;

public class PickUpGamePiece extends AutoCommand {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final JointSubsystem jointSubsystem;
    private Timer timer = new Timer();
    private boolean gotPiece = false;

    public PickUpGamePiece(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem,
            JointSubsystem jointSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.jointSubsystem = jointSubsystem;
        addRequirements(drivetrainSubsystem);
        addRequirements(intakeSubsystem);
        addRequirements(jointSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        gotPiece = false;
    }

    @Override
    public void execute() {
        if (gotPiece) {
            drivetrainSubsystem.stopMotion();
            return;
        }
        if (intakeSubsystem.pieceDetected()) {
            jointSubsystem.setPosition(JOINT_CLAMP_POSITION);
        }
        drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0.5, 0, 0));
        if (timer.hasElapsed(0.5)) {
            if (Math.abs(intakeSubsystem.getSmoothCurrent()) > INTAKE_THRESHOLD_CURRENT) {
                gotPiece = true;
                intakeSubsystem.setMotorDutyCycle(-0.08);
                jointSubsystem.setPosition(JOINT_STOW_POSITION);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2.0) || gotPiece;
    }

    @Override
    public boolean didSucceed() {
        return gotPiece;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopMotion();
    }
}
