package com.team2383.robot.commands.blizzard;

import com.team2383.robot.commands.ElevatorPositionCommand;
import com.team2383.robot.commands.WristPositionCommand;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class BlizzardCommand extends ParallelCommandGroup {
    public BlizzardCommand(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            BlizzardPosition position) {

        addCommands(new ElevatorPositionCommand(elevatorSubsystem, position.getExtension()),
                new WristPositionCommand(wristSubsystem, position.getWristAngle()));
    }

}
