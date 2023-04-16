package frc.robot.commands.positions;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.commands.SleepCommand;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

import static frc.robot.Constants.*;

public class HighNodeCommand extends DynamicCommand{

    private final ElevatorSubsystem elevatorSubsystem;
    private final TelescopeSubsystem telescopeSubsystem;
    private final JointSubsystem jointSubsystem;
    private final LEDStripSubsystem ledStripSubsystem;

    private final BooleanSupplier cubeMode;

    public HighNodeCommand(ElevatorSubsystem elevatorSubsystem, TelescopeSubsystem telescopeSubsystem, JointSubsystem jointSubsystem, LEDStripSubsystem ledStripSubsystem, BooleanSupplier cubeMode) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.telescopeSubsystem = telescopeSubsystem;
        this.jointSubsystem = jointSubsystem;
        this.ledStripSubsystem = ledStripSubsystem;
        this.cubeMode = cubeMode;

        addRequirements(elevatorSubsystem, telescopeSubsystem, jointSubsystem, ledStripSubsystem);
    }

    @Override
    protected Command getCommand() {
        if(!(elevatorSubsystem.isZeroed() && telescopeSubsystem.isZeroed() && jointSubsystem.isZeroed())) {
            return Commands.parallel(
                elevatorSubsystem.zeroCommand(),
                telescopeSubsystem.zeroCommand(),
                jointSubsystem.zeroCommand(),
                ledStripSubsystem.setPatternCommand(-0.11) // Strobe Red
            );
        }
        if(elevatorSubsystem.getPosition() < (cubeMode.getAsBoolean()? ELEVATOR_CUBE_LOW_POSITION: ELEVATOR_CONE_LOW_POSITION)) {
            // Going Up
            return new SequentialCommandGroup(
                elevatorSubsystem.highNodeCommand(cubeMode),
                new SleepCommand(0.5),
                telescopeSubsystem.highNodeCommand(cubeMode),
                jointSubsystem.highNodeCommand(cubeMode)
            );
        }else {
            // Going Down
            return new SequentialCommandGroup(
                telescopeSubsystem.highNodeCommand(cubeMode),
                jointSubsystem.highNodeCommand(cubeMode),
                new SleepCommand(0.3),
                elevatorSubsystem.highNodeCommand(cubeMode)
            );
        }
    }
    
}
