package com.team2383.robot.subsystems.drivetrain.SLAM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface SLAMIO {
    public static class SLAMIOInputs {
        public boolean connected = false;
        public boolean newValue = false;
        public Pose3d pose = new Pose3d();
        public Pose3d[] landmarks = new Pose3d[0];
        public Pose3d[] seenLandmarks = new Pose3d[0];
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(SLAMIOInputs inputs) {
    }

    public default void updateModulePositions(SwerveModulePosition[] modulePositions,
            Rotation2d gyroAngle) {
    }

    public default void setVisionConstants(Transform3d[] camPoses, double varianceScale, double varianceStatic) {
    }

    public default void forceHeading(Rotation2d heading, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
    }

    public default void forcePose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
    }

    public default void saveAndExit(boolean saveAndExit) {
    }
}
