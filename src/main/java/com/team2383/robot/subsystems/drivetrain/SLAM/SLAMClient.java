package com.team2383.robot.subsystems.drivetrain.SLAM;

import com.team2383.robot.subsystems.drivetrain.SLAM.SLAMIO.SLAMIOInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;

public class SLAMClient {
    private final SLAMIO slamio;
    private final SLAMIOInputs inputs = new SLAMIOInputs();

    public SLAMClient(SLAMIO io) {
        slamio = io;
    }

    public void seedSLAMLandmarks(Pose3d[] landmarks) {
        slamio.seedLandmarks(landmarks);
    }

    public void setVisionConstants(Transform3d[] camPoses, double varianceScale, double varianceStatic) {
        slamio.setVisionConstants(camPoses, varianceScale, varianceStatic);
    }

    public SLAMUpdate update(SwerveModulePosition[] positions, Rotation2d gyroAngle) {
        slamio.updateModulePositions(positions, gyroAngle);

        slamio.updateInputs(inputs);

        slamio.saveAndExit(DriverStation.getStickButton(0, 1));

        return new SLAMUpdate(inputs.pose, inputs.landmarks, inputs.seenLandmarks, inputs.newValue);
    }

    public void forceOdometry(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        slamio.forcePose(pose, gyroAngle, positions);
    }

    public void forceHeading(Rotation2d heading, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        slamio.forceHeading(heading, gyroAngle, positions);
    }
}
