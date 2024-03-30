package com.team2383.robot.subsystems.drivetrain.SLAM;

import org.littletonrobotics.junction.Logger;

import com.team2383.lib.CameraParameters;
import com.team2383.lib.util.mechanical_advantage.LoggedTunableNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SLAMClient {
    private final SLAMIO slamio;
    private final SLAMIOInputsAutoLogged inputs = new SLAMIOInputsAutoLogged();

    private final LoggedTunableNumber cameraExposeAuto = new LoggedTunableNumber("SLAM/Camera/ExposeAuto", 1);
    private final LoggedTunableNumber cameraExpose = new LoggedTunableNumber("SLAM/Camera/Expose", 100);
    private final LoggedTunableNumber cameraGain = new LoggedTunableNumber("SLAM/Camera/Gain", 0);

    private Transform3d[] camPoses;
    private double varianceScale;
    private double varianceStatic;

    public SLAMClient(SLAMIO io) {
        slamio = io;
    }

    public void setVisionConstants(Transform3d[] camPoses, double varianceScale, double varianceStatic) {
        this.camPoses = camPoses;
        this.varianceScale = varianceScale;
        this.varianceStatic = varianceStatic;
        slamio.setVisionConstants(camPoses, varianceScale, varianceStatic,
                new CameraParameters(1280, 720,
                        (int) cameraExposeAuto.getAsDouble(),
                        (int) cameraExpose.getAsDouble(),
                        (int) cameraGain.getAsDouble()));
    }

    public SLAMUpdate update(SwerveModulePosition[] positions, Rotation2d gyroAngle) {
        slamio.updateModulePositions(positions, gyroAngle);

        slamio.updateInputs(inputs);

        Logger.processInputs("SLAM", inputs);

        // slamio.saveAndExit(DriverStation.getStickButton(0, 1));

        LoggedTunableNumber.ifChanged(this.hashCode(),
                () -> setVisionConstants(camPoses, varianceScale, varianceStatic),
                cameraExposeAuto, cameraExpose, cameraGain);

        return new SLAMUpdate(inputs.pose, inputs.landmarks, inputs.seenLandmarks, inputs.newValue);
    }

    public void forceOdometry(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        slamio.forcePose(pose, gyroAngle, positions);
    }

    public void forceHeading(Rotation2d heading, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        slamio.forceHeading(heading, gyroAngle, positions);
    }
}
