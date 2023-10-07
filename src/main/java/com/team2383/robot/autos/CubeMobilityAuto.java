package com.team2383.robot.autos;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.robot.commands.blizzard.BlizzardCommand;
import com.team2383.robot.commands.blizzard.BlizzardPresets;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CubeMobilityAuto extends SequentialCommandGroup {
    public CubeMobilityAuto(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, WristSubsystem wrist,
            String pathName, SwerveAutoBuilder autoBuilder) {

        addCommands(
                new BlizzardCommand(elevator, wrist, BlizzardPresets.MIDDLE),
                new FullAutoCommand(drivetrain, "Path1", autoBuilder));
    }
}
