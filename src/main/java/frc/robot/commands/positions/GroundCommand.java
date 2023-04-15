package frc.robot.commands.positions;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.commands.SleepCommand;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

import static frc.robot.Constants.*;

public class GroundCommand extends DynamicCommand{

    private final ElevatorSubsystem elevatorSubsystem;
    private final TelescopeSubsystem telescopeSubsystem;
    private final JointSubsystem jointSubsystem;

    private final BooleanSupplier cubeMode;

    public GroundCommand(ElevatorSubsystem elevatorSubsystem, TelescopeSubsystem telescopeSubsystem, JointSubsystem jointSubsystem, BooleanSupplier cubeMode) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.telescopeSubsystem = telescopeSubsystem;
        this.jointSubsystem = jointSubsystem;
        this.cubeMode = cubeMode;

        addRequirements(elevatorSubsystem, telescopeSubsystem, jointSubsystem);
    }

    @Override
    protected Command getCommand() {
        if(elevatorSubsystem.getPosition() < (cubeMode.getAsBoolean()? ELEVATOR_CUBE_GROUND_POSITION: ELEVATOR_CONE_GROUND_POSITION)) {
            // Going Up
            return new SequentialCommandGroup(
                elevatorSubsystem.groundCommand(cubeMode),
                new SleepCommand(0.3),
                telescopeSubsystem.groundCommand(cubeMode),
                jointSubsystem.groundCommand(cubeMode)
            );
        }else {
            // Going Down
            return new SequentialCommandGroup(
                telescopeSubsystem.groundCommand(cubeMode),
                jointSubsystem.groundCommand(cubeMode),
                new SleepCommand(0.3),
                elevatorSubsystem.groundCommand(cubeMode)
            );
        }
    }
    
}
