package com.team2383.robot.commands.drivetrain;

import com.team2383.robot.FieldConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FaceToSpeakerCommand extends SequentialCommandGroup {

    public FaceToSpeakerCommand(DrivetrainSubsystem drivetrain) {
        Translation2d speakerLocation;

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            speakerLocation = FieldConstants.BLUE_SPEAKER;
        } else {
            speakerLocation = FieldConstants.RED_SPEAKER;
        }

        addCommands(
                new FaceToTranslationCommand(drivetrain, speakerLocation));
    }

}
