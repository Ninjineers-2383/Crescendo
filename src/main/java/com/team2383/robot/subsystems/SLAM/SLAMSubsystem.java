package com.team2383.robot.subsystems.SLAM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.*;

public class SLAMSubsystem extends SubsystemBase {
    private final SLAMIO slamio;
    private final SLAMIOInputsAutoLogged inputs = new SLAMIOInputsAutoLogged();

    private Consumer<SLAMUpdate> SLAMUpdateConsumer;

    public SLAMSubsystem(SLAMIO io) {
        slamio = io;
    }

    public void SLAMupdateSubscribe(Consumer<SLAMUpdate> consumer) {
        SLAMUpdateConsumer = consumer;
    }

    public void seedSLAMLandmarks(Pose3d[] landmarks) {
        slamio.seedLandmarks(landmarks);
    }

    @Override
    public void periodic() {
        slamio.updateInputs(inputs);

        Logger.processInputs("SLAM", inputs);

        if (inputs.newValue) {
            onSLAMUpdate(inputs.pose, inputs.landmarks, inputs.seenLandmarks);
        }
    }

    public void odometryConsumer(ChassisSpeeds speeds, SwerveModulePosition[] positions, Rotation2d gyroAngle) {
        slamio.updateChassisSpeeds(speeds, positions, gyroAngle);
    }

    private void onSLAMUpdate(Pose3d pose, Pose3d[] landmarks, Pose3d[] seenLandmarks) {
        SLAMUpdateConsumer.accept(new SLAMUpdate(pose, landmarks, seenLandmarks));
    }
}
