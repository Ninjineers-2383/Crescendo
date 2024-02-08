package com.team2383.lib.util;

import com.team2383.robot.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceUtil {

    public static Translation2d flipTranslation2d(Translation2d translation) {
        if (!DriverStation.getAlliance().isPresent()) {
            return translation;
        }

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return new Translation2d(FieldConstants.fieldLength - translation.getX(), translation.getY());
        } else {
            return translation;
        }
    }

    public static Rotation2d flipRotation2d(Rotation2d rotation) {
        if (!DriverStation.getAlliance().isPresent()) {
            return rotation;
        }

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return rotation.rotateBy(Rotation2d.fromDegrees(180));
        } else {
            return rotation;
        }
    }

    public static Translation3d flipTranslation3d(Translation3d translation) {
        if (!DriverStation.getAlliance().isPresent()) {
            return translation;
        }

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return new Translation3d(FieldConstants.fieldLength - translation.getX(), translation.getY(),
                    translation.getZ());
        } else {
            return translation;
        }
    }

    public static Rotation3d flipRotation3d(Rotation3d rotation) {
        if (!DriverStation.getAlliance().isPresent()) {
            return rotation;
        }

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return rotation.rotateBy(new Rotation3d(180, 0, 0)); // TODO: unit test
        } else {
            return rotation;
        }
    }

    public static Pose2d flipPose2d(Pose2d pose) {
        return new Pose2d(flipTranslation2d(pose.getTranslation()), flipRotation2d(pose.getRotation()));
    }

    public static Pose3d flipPose3d(Pose3d pose) {
        return new Pose3d(flipTranslation3d(pose.getTranslation()), flipRotation3d(pose.getRotation()));
    }

}
