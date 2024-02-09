package com.team2383.robot.commands;

import com.team2383.robot.commands.drivetrain.FaceToSpeakerCommand;
import com.team2383.robot.commands.pivot.PivotSeekCommand;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class SeekCommand extends ParallelCommandGroup {

    public SeekCommand(DrivetrainSubsystem drivetrain, PivotSubsystem pivot) {

        addCommands(
                new FaceToSpeakerCommand(drivetrain),
                // new ShooterRPMCommand(shooter, () -> 6800, () -> 1000, () -> 0),
                new PivotSeekCommand(pivot, drivetrain::getEstimatorPose3d));
    }

}
