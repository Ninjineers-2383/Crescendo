package com.team2383.robot.commands.drivetrain;

import com.team2383.robot.FieldConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FaceToSpeakerCommand extends SequentialCommandGroup {

    public FaceToSpeakerCommand(DrivetrainSubsystem drivetrain) {

        addCommands(
                new FaceToTranslationCommand(drivetrain, FieldConstants.getSpeakerLocation()));
    }

}
