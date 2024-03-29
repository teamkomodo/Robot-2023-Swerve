package frc.robot.commands.positions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.commands.SleepCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class StowCommand extends SequentialCommandGroup{
    
    public StowCommand(ElevatorSubsystem elevatorSubsystem, TelescopeSubsystem telescopeSubsystem, JointSubsystem jointSubsystem) {
        addCommands(
            telescopeSubsystem.stowCommand(),
            jointSubsystem.stowCommand(),
            new SleepCommand(0.3),
            elevatorSubsystem.stowCommand(),
            new SleepCommand(0.2)
        );
    }
}
