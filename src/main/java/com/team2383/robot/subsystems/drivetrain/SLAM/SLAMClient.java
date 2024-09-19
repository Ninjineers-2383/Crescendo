package com.team2383.robot.subsystems.drivetrain.SLAM;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.team2383.lib.CameraParameters;
import com.team2383.lib.util.mechanical_advantage.LoggedTunableNumber;
import com.team2383.robot.subsystems.drivetrain.vision.VisionIONorthstar;
import com.team2383.robot.subsystems.drivetrain.vision.VisionSubsystem;
import com.team2383.robot.subsystems.drivetrain.vision.VisionSubsystem.TimestampVisionUpdate;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SLAMClient {
    private final SwerveDrivePoseEstimator estimator;

    private VisionSubsystem vision;

    private final LoggedTunableNumber[] cameraExposeAuto;
    private final LoggedTunableNumber[] cameraExpose;
    private final LoggedTunableNumber[] cameraGain;

    private Transform3d[] camPoses;
    private Pose3d[] landmarks;
    private double varianceScale;
    private double varianceStatic;

    private final Supplier<Pose3d> poseSupplier;

    private boolean[] seenTags = new boolean[16];

    public SLAMClient(SwerveDrivePoseEstimator estimator, Pose3d[] landmarks, Supplier<Pose3d> poseSupplier) {
        this.estimator = estimator;
        this.landmarks = landmarks;
        this.poseSupplier = poseSupplier;
        cameraExposeAuto = new LoggedTunableNumber[] {
                new LoggedTunableNumber("SLAM/Camera/0/ExposeAuto", 1),
                new LoggedTunableNumber("SLAM/Camera/1/ExposeAuto", 1),
                new LoggedTunableNumber("SLAM/Camera/2/ExposeAuto", 1),
                new LoggedTunableNumber("SLAM/Camera/3/ExposeAuto", 1)
        };

        cameraGain = new LoggedTunableNumber[] {
                new LoggedTunableNumber("SLAM/Camera/0/Gain", 10),
                new LoggedTunableNumber("SLAM/Camera/1/Gain", 10),
                new LoggedTunableNumber("SLAM/Camera/2/Gain", 10),
                new LoggedTunableNumber("SLAM/Camera/3/Gain", 10)
        };

        cameraExpose = new LoggedTunableNumber[] {
                new LoggedTunableNumber("SLAM/Camera/0/Exposure", 50),
                new LoggedTunableNumber("SLAM/Camera/1/Exposure", 25),
                new LoggedTunableNumber("SLAM/Camera/2/Exposure", 25),
                new LoggedTunableNumber("SLAM/Camera/3/Exposure", 50)
        };
    }

    public void setVisionConstants(Transform3d[] camPoses, double varianceScale, double varianceStatic) {
        this.camPoses = camPoses;
        this.varianceScale = varianceScale;
        this.varianceStatic = varianceStatic;
        CameraParameters[] params = getParameters();
        vision = new VisionSubsystem(landmarks,
                new VisionIONorthstar("northstar-1", params[0]),
                new VisionIONorthstar("northstar-2", params[1]),
                new VisionIONorthstar("northstar-3", params[2]),
                new VisionIONorthstar("northstar-4", params[3]));

        vision.setVisionConstants(camPoses, varianceScale, varianceStatic);
        vision.setVisionConsumer(this::visionConsumer);
        vision.setPoseSupplier(poseSupplier);
    }

    public SLAMUpdate update(SwerveModulePosition[] positions, Rotation2d gyroAngle) {
        vision.periodic();

        estimator.update(gyroAngle, positions);

        LoggedTunableNumber.ifChanged(this.hashCode(),
                () -> setVisionConstants(camPoses, varianceScale, varianceStatic),
                cameraExposeAuto[0], cameraExposeAuto[1], cameraExposeAuto[2], cameraExposeAuto[3],
                cameraExpose[0], cameraExpose[1], cameraExpose[2], cameraExpose[3],
                cameraGain[0], cameraGain[1], cameraGain[2], cameraGain[3]);

        Logger.recordOutput("SeenTags", seenTags);

        ArrayList<Pose3d> seenTagPositions = new ArrayList<>();
        for (int i = 0; i < seenTags.length; i++) {
            if (seenTags[i]) {
                seenTagPositions.add(landmarks[i]);
            }
        }

        Pose3d[] seenTagPosesArray = new Pose3d[seenTagPositions.size()];
        Pose2d[] seenTagPosesArray2d = new Pose2d[seenTagPositions.size()];

        for (int i = 0; i < seenTagPositions.size(); i++) {
            seenTagPosesArray[i] = seenTagPositions.get(i);
            seenTagPosesArray2d[i] = seenTagPositions.get(i).toPose2d();
        }

        Logger.recordOutput("SLAM/SeenLandmarkPoses", seenTagPosesArray);
        Logger.recordOutput("SLAM/SeenLandmarkPoses2D", seenTagPosesArray2d);

        seenTags = new boolean[16];

        return new SLAMUpdate(new Pose3d(estimator.getEstimatedPosition()), new Pose3d[0], 0, true);
    }

    public void forceOdometry(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        estimator.resetPosition(gyroAngle, positions, pose);
    }

    public void forceHeading(Rotation2d heading, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        estimator.resetPosition(gyroAngle, positions,
                new Pose2d(estimator.getEstimatedPosition().getTranslation(), heading));
    }

    public void visionConsumer(List<TimestampVisionUpdate> vision) {
        for (TimestampVisionUpdate measurement : vision) {
            Pose2d pose = landmarks[measurement.tagId1() - 1]
                    .plus(measurement.pose().inverse())
                    .toPose2d();
            estimator.addVisionMeasurement(pose, measurement.timestamp());
            seenTags[measurement.tagId1() - 1] = true;
            if (measurement.tagId2() != 0) {
                seenTags[measurement.tagId2() - 1] = true;
            }
        }
    }

    private CameraParameters[] getParameters() {
        return new CameraParameters[] {
                new CameraParameters(1280, 720,
                        (int) cameraExposeAuto[0].getAsDouble(),
                        (int) cameraExpose[0].getAsDouble(),
                        (int) cameraGain[0].getAsDouble()),
                new CameraParameters(1280, 720,
                        (int) cameraExposeAuto[1].getAsDouble(),
                        (int) cameraExpose[1].getAsDouble(),
                        (int) cameraGain[1].getAsDouble()),
                new CameraParameters(1280, 720,
                        (int) cameraExposeAuto[2].getAsDouble(),
                        (int) cameraExpose[2].getAsDouble(),
                        (int) cameraGain[2].getAsDouble()),
                new CameraParameters(1280, 720,
                        (int) cameraExposeAuto[3].getAsDouble(),
                        (int) cameraExpose[3].getAsDouble(),
                        (int) cameraGain[3].getAsDouble())
        };
    }
}
