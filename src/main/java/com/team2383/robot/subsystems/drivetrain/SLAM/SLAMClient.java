package com.team2383.robot.subsystems.drivetrain.SLAM;

import com.team2383.robot.subsystems.drivetrain.SLAM.SLAMIO.SLAMIOInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SLAMClient {
    private final SLAMIO slamio;
    private final SLAMIOInputs inputs = new SLAMIOInputs();

    public SLAMClient(SLAMIO io) {
        slamio = io;
    }

    public void seedSLAMLandmarks(Pose3d[] landmarks) {
        slamio.seedLandmarks(landmarks);
    }

    public SLAMUpdate update(ChassisSpeeds speeds, SwerveModulePosition[] positions, Rotation2d gyroAngle) {
        slamio.updateChassisSpeeds(speeds, positions, gyroAngle);

        slamio.updateInputs(inputs);

        return new SLAMUpdate(inputs.pose, inputs.landmarks, inputs.seenLandmarks);
    }

    public void forceOdometry(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        slamio.forcePose(pose, gyroAngle, positions);
    }

    public void forceHeading(Rotation2d heading, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        slamio.forceHeading(heading, gyroAngle, positions);
    }
}
