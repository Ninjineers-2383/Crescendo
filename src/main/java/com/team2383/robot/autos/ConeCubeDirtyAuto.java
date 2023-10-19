package com.team2383.robot.autos;

import com.team2383.robot.autos.auto_blocks.FullAutoCommand;
import com.team2383.robot.autos.auto_blocks.ScoreHighCommand;
import com.team2383.robot.commands.blizzard.BlizzardCommand;
import com.team2383.robot.commands.blizzard.BlizzardPresets;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeCubeDirtyAuto extends SequentialCommandGroup {
    public ConeCubeDirtyAuto(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, WristSubsystem wrist,
            FeederSubsystem feeder, String pathName) {

        addCommands(
                new ScoreHighCommand(elevator, wrist, feeder, false),
                new FullAutoCommand(drivetrain, elevator, wrist, feeder, pathName),
                new BlizzardCommand(elevator, wrist, BlizzardPresets.CONE_CHUTE));
    }
}
