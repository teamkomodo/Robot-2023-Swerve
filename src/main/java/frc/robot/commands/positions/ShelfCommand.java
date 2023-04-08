package frc.robot.commands.positions;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DynamicCommand;
import frc.robot.commands.auto.SleepCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

import static frc.robot.Constants.*;

public class ShelfCommand extends DynamicCommand{

    private final ElevatorSubsystem elevatorSubsystem;
    private final TelescopeSubsystem telescopeSubsystem;
    private final JointSubsystem jointSubsystem;
    private final ClawSubsystem clawSubsystem;

    private final BooleanSupplier cubeMode;

    public ShelfCommand(ElevatorSubsystem elevatorSubsystem, TelescopeSubsystem telescopeSubsystem, JointSubsystem jointSubsystem, ClawSubsystem clawSubsystem, BooleanSupplier cubeMode) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.telescopeSubsystem = telescopeSubsystem;
        this.jointSubsystem = jointSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.cubeMode = cubeMode;

        addRequirements(elevatorSubsystem, telescopeSubsystem, jointSubsystem, clawSubsystem);
    }

    @Override
    protected Command getCommand() {
        if(elevatorSubsystem.getMotorPosition() < (cubeMode.getAsBoolean()? ELEVATOR_CUBE_SHELF_POSITION: ELEVATOR_CONE_SHELF_POSITION)) {
            // Going Up
            return new SequentialCommandGroup(
                elevatorSubsystem.shelfCommand(cubeMode),
                new SleepCommand(0.3),
                telescopeSubsystem.shelfCommand(cubeMode),
                jointSubsystem.shelfCommand(cubeMode),
                clawSubsystem.openCommand()
            );
        }else {
            // Going Down
            return new SequentialCommandGroup(
                telescopeSubsystem.shelfCommand(cubeMode),
                jointSubsystem.shelfCommand(cubeMode),
                clawSubsystem.openCommand(),
                new SleepCommand(0.3),
                elevatorSubsystem.shelfCommand(cubeMode)
            );
        }
    }
    
}
