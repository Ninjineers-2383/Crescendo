package com.team2383.robot.autos;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.robot.commands.FeederVoltageCommand;
import com.team2383.robot.commands.blizzard.BlizzardCommand;
import com.team2383.robot.commands.blizzard.BlizzardPresets;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CubeConeAutoClean extends SequentialCommandGroup {
    public CubeConeAutoClean(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, WristSubsystem wrist,
            FeederSubsystem feeder, String pathName, SwerveAutoBuilder autoBuilder) {

        addCommands(
                new InstantCommand(() -> drivetrain.forceOdometry(new Pose2d(1.81, 4.39, Rotation2d.fromDegrees(180)))),
                new FeederVoltageCommand(feeder, () -> 0, true).withTimeout(0.5),
                new BlizzardCommand(elevator, wrist, BlizzardPresets.HIGH).withTimeout(2),
                new FeederVoltageCommand(feeder, () -> 0.2, true).withTimeout(0.5),
                new BlizzardCommand(elevator, wrist, BlizzardPresets.CONE_CHUTE).withTimeout(2),
                new FullAutoCommand(drivetrain, pathName, autoBuilder),
                new BlizzardCommand(elevator, wrist, BlizzardPresets.HIGH).withTimeout(2),
                new FeederVoltageCommand(feeder, () -> 0.2, false).withTimeout(0.5));

    }
}
