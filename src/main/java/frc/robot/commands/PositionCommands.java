package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.commands.SleepCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

import static frc.robot.Constants.*;

public class PositionCommands {

    public static Command lowNodeCommand(ElevatorSubsystem elevatorSubsystem, TelescopeSubsystem telescopeSubsystem, ClawSubsystem clawSubsystem, JointSubsystem jointSubsystem, BooleanSupplier cubeMode) {
        
        SequentialCommandGroup upGroup = new SequentialCommandGroup(
            elevatorSubsystem.lowNodeCommand(cubeMode),
            new SleepCommand(0.3),
            telescopeSubsystem.lowNodeCommand(cubeMode),
            jointSubsystem.lowNodeCommand(cubeMode)
        );

        SequentialCommandGroup downGroup = new SequentialCommandGroup(
            telescopeSubsystem.lowNodeCommand(cubeMode),
            jointSubsystem.lowNodeCommand(cubeMode),
            new SleepCommand(0.3),
            elevatorSubsystem.lowNodeCommand(cubeMode)
        );

        return Commands.runOnce(() -> {
            if(elevatorSubsystem.getPosition() < (cubeMode.getAsBoolean()? ELEVATOR_CUBE_LOW_POSITION : ELEVATOR_CONE_LOW_POSITION)) {
                CommandScheduler.getInstance().schedule(upGroup);
            }else {
                CommandScheduler.getInstance().schedule(downGroup);
            }
        });
    }

    public static Command midNodeCommand(ElevatorSubsystem elevatorSubsystem, TelescopeSubsystem telescopeSubsystem, ClawSubsystem clawSubsystem, JointSubsystem jointSubsystem, BooleanSupplier cubeMode) {
        
        SequentialCommandGroup upGroup = new SequentialCommandGroup(
            elevatorSubsystem.midNodeCommand(cubeMode),
            new SleepCommand(0.3),
            telescopeSubsystem.midNodeCommand(cubeMode),
            jointSubsystem.midNodeCommand(cubeMode)
        );

        SequentialCommandGroup downGroup = new SequentialCommandGroup(
            telescopeSubsystem.midNodeCommand(cubeMode),
            jointSubsystem.midNodeCommand(cubeMode),
            new SleepCommand(0.3),
            elevatorSubsystem.midNodeCommand(cubeMode)
        );

        return Commands.runOnce(() -> {
            if(elevatorSubsystem.getPosition() < (cubeMode.getAsBoolean()? ELEVATOR_CUBE_MID_POSITION : ELEVATOR_CONE_MID_POSITION)) {
                CommandScheduler.getInstance().schedule(upGroup);
            }else {
                CommandScheduler.getInstance().schedule(downGroup);
            }
        });
    }

    public static Command highNodeCommand(ElevatorSubsystem elevatorSubsystem, TelescopeSubsystem telescopeSubsystem, ClawSubsystem clawSubsystem, JointSubsystem jointSubsystem, BooleanSupplier cubeMode) {
        
        SequentialCommandGroup upGroup = new SequentialCommandGroup(
            elevatorSubsystem.highNodeCommand(cubeMode),
            new SleepCommand(0.3),
            telescopeSubsystem.highNodeCommand(cubeMode),
            jointSubsystem.highNodeCommand(cubeMode)
        );

        SequentialCommandGroup downGroup = new SequentialCommandGroup(
            telescopeSubsystem.highNodeCommand(cubeMode),
            jointSubsystem.highNodeCommand(cubeMode),
            new SleepCommand(0.3),
            elevatorSubsystem.highNodeCommand(cubeMode)
        );

        return Commands.runOnce(() -> {
            if(elevatorSubsystem.getPosition() < (cubeMode.getAsBoolean()? ELEVATOR_CUBE_HIGH_POSITION : ELEVATOR_CONE_HIGH_POSITION)) {
                CommandScheduler.getInstance().schedule(upGroup);
            }else {
                CommandScheduler.getInstance().schedule(downGroup);
            }
        });
    }

    public static Command shelfCommand(ElevatorSubsystem elevatorSubsystem, TelescopeSubsystem telescopeSubsystem, ClawSubsystem clawSubsystem, JointSubsystem jointSubsystem, BooleanSupplier cubeMode) {
        
        SequentialCommandGroup upGroup = new SequentialCommandGroup(
            elevatorSubsystem.shelfCommand(cubeMode),
            new SleepCommand(0.3),
            telescopeSubsystem.shelfCommand(cubeMode),
            jointSubsystem.shelfCommand(cubeMode)
        );

        SequentialCommandGroup downGroup = new SequentialCommandGroup(
            telescopeSubsystem.shelfCommand(cubeMode),
            jointSubsystem.shelfCommand(cubeMode),
            new SleepCommand(0.3),
            elevatorSubsystem.shelfCommand(cubeMode)
        );

        return Commands.runOnce(() -> {
            if(elevatorSubsystem.getPosition() < (cubeMode.getAsBoolean()? ELEVATOR_CUBE_SHELF_POSITION : ELEVATOR_CONE_SHELF_POSITION)) {
                CommandScheduler.getInstance().schedule(upGroup);
            }else {
                CommandScheduler.getInstance().schedule(downGroup);
            }
        });
    }

    public static Command stowCommand(ElevatorSubsystem elevatorSubsystem, TelescopeSubsystem telescopeSubsystem, ClawSubsystem clawSubsystem, JointSubsystem jointSubsystem, BooleanSupplier cubeMode) {
        return new SequentialCommandGroup(
            telescopeSubsystem.stowCommand(),
            jointSubsystem.stowCommand(),
            clawSubsystem.closeCommand(),
            new SleepCommand(0.3),
            elevatorSubsystem.stowCommand()
        );
    }
}