package com.team2383.robot.commands.speaker;

import com.team2383.robot.commands.subsystem.drivetrain.FaceToSpeakerCommand;
import com.team2383.robot.commands.subsystem.pivot.PivotSeekCommand;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class SeekCommand extends ParallelCommandGroup {

    public SeekCommand(DrivetrainSubsystem drivetrain, PivotSubsystem pivot, boolean finish) {

        addCommands(
                new FaceToSpeakerCommand(drivetrain, finish),
                new PivotSeekCommand(pivot, drivetrain::getEstimatorPose3d));
    }

}
