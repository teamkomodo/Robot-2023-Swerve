package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.commands.SleepCommand;
import frc.robot.commands.positions.GroundCommand;
import frc.robot.commands.positions.HighNodeCommand;
import frc.robot.commands.positions.StowCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class CommandFactory {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final TelescopeSubsystem telescopeSubsystem;
    private final JointSubsystem jointSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LEDStripSubsystem ledStripSubsystem;

    public CommandFactory(ElevatorSubsystem elevatorSubsystem, TelescopeSubsystem telescopeSubsystem, JointSubsystem jointSubsystem, IntakeSubsystem intakeSubsystem, LEDStripSubsystem ledStripSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.telescopeSubsystem = telescopeSubsystem;
        this.jointSubsystem = jointSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.ledStripSubsystem = ledStripSubsystem;
    }

    public Command getEjectCommand() {
        return Commands.runEnd(() -> {
                ledStripSubsystem.setPattern(-0.01); // Color 1 Larson Scanner
                intakeSubsystem.setMotorDutyCycle(1.0);
            }, () -> {
                intakeSubsystem.setMotorDutyCycle(0);
            }, intakeSubsystem, ledStripSubsystem);
    }

    public Command getPlaceHighCommand(BooleanSupplier cubeMode) {
        return Commands.sequence(
            new HighNodeCommand(elevatorSubsystem, telescopeSubsystem, jointSubsystem, ledStripSubsystem, cubeMode),
            new SleepCommand(1.0),
            getEjectCommand().withTimeout(0.2),
            new StowCommand(elevatorSubsystem, telescopeSubsystem, jointSubsystem));
    }

    public Command getPickupGroundCommand(BooleanSupplier cubeMode) {
        return Commands.sequence(
            new GroundCommand(elevatorSubsystem, telescopeSubsystem, jointSubsystem, ledStripSubsystem, cubeMode),
            new IntakePieceCommand(intakeSubsystem, jointSubsystem, ledStripSubsystem).withTimeout(3.0)
        );
    }

}
