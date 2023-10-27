package com.team2383.robot.autos.auto_blocks;

import com.team2383.robot.commands.FeederVoltageCommand;
import com.team2383.robot.commands.blizzard.BlizzardCommand;
import com.team2383.robot.commands.blizzard.BlizzardPresets;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class FeedCone extends ParallelCommandGroup {
    public FeedCone(ElevatorSubsystem elevator, WristSubsystem wrist, FeederSubsystem feeder) {
        addCommands(
                new BlizzardCommand(elevator, wrist, BlizzardPresets.GROUND_INTAKE),
                new FeederVoltageCommand(feeder, () -> -0.5, false));
    }
}
