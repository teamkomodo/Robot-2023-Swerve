package frc.robot.commands.positions;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.commands.SleepCommand;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

import static frc.robot.Constants.*;

public class MidNodeCommand extends DynamicCommand{

    private final ElevatorSubsystem elevatorSubsystem;
    private final TelescopeSubsystem telescopeSubsystem;
    private final JointSubsystem jointSubsystem;

    private final BooleanSupplier cubeMode;

    public MidNodeCommand(ElevatorSubsystem elevatorSubsystem, TelescopeSubsystem telescopeSubsystem, JointSubsystem jointSubsystem, BooleanSupplier cubeMode) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.telescopeSubsystem = telescopeSubsystem;
        this.jointSubsystem = jointSubsystem;
        this.cubeMode = cubeMode;

        addRequirements(elevatorSubsystem, telescopeSubsystem, jointSubsystem);
    }

    @Override
    protected Command getCommand() {
        if(!(elevatorSubsystem.isZeroed() && telescopeSubsystem.isZeroed() && jointSubsystem.isZeroed())) {
            return new InstantCommand();
        }
        if(elevatorSubsystem.getPosition() < (cubeMode.getAsBoolean()? ELEVATOR_CUBE_LOW_POSITION: ELEVATOR_CONE_LOW_POSITION)) {
            // Going Up
            return new SequentialCommandGroup(
                elevatorSubsystem.midNodeCommand(cubeMode),
                new SleepCommand(0.3),
                telescopeSubsystem.midNodeCommand(cubeMode),
                jointSubsystem.midNodeCommand(cubeMode)
            );
        }else {
            // Going Down
            return new SequentialCommandGroup(
                telescopeSubsystem.midNodeCommand(cubeMode),
                jointSubsystem.midNodeCommand(cubeMode),
                new SleepCommand(0.3),
                elevatorSubsystem.midNodeCommand(cubeMode)
            );
        }
    }
    
}
