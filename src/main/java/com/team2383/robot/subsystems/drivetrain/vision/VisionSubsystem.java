package com.team2383.robot.subsystems.drivetrain.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.List;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import com.team2383.robot.subsystems.drivetrain.vision.VisionIO.VisionIOInputs;

public class VisionSubsystem {
    private final VisionIO[] visionIO;
    private final VisionIOInputs[] inputs;

    private Consumer<List<TimestampVisionUpdate>> visionConsumer = (x) -> {
    };
    private Supplier<Pose3d> poseSupplier = () -> new Pose3d();

    private Pose3d[] tagPoses;

    private Transform3d[] camTransforms;
    private double varianceScale;
    private double varianceStatic;

    private boolean enableSingleTag = false;

    public VisionSubsystem(Pose3d[] tagPoses, VisionIO... visionIO) {
        this.tagPoses = tagPoses;
        this.visionIO = visionIO;
        this.inputs = new VisionIOInputs[visionIO.length];
        for (int i = 0; i < visionIO.length; i++) {
            inputs[i] = new VisionIOInputs();
            visionIO[i].setTagPoses(tagPoses);
        }
    }

    public void periodic() {
        Pose3d robotPose = poseSupplier.get();
        for (int i = 0; i < visionIO.length; i++) {
            visionIO[i].updateInputs(inputs[i], robotPose);
            Logger.processInputs("Camera/" + String.valueOf(i), inputs[i]);
        }

        List<TimestampVisionUpdate> updates = new ArrayList<>();
        for (int i = 0; i < Math.min(inputs.length, camTransforms.length); i++) {
            for (int j = 0; j < inputs[i].frames.length; j++) {
                double timestamp = inputs[i].timestamps[j];

                double[] values = inputs[i].frames[j];

                switch ((int) values[0]) {
                    case 1: // 1 pose estimate
                        Pose3d pose = new Pose3d(
                                values[2], values[3], values[4],
                                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                        int tag1 = (int) values[9], tag2 = (int) values[10];
                        Transform3d offset = new Transform3d(pose, tagPoses[tag1 - 1])
                                .plus(new Transform3d(pose, tagPoses[tag2 - 1])).div(2);

                        if (Math.abs(tagPoses[tag1 - 1].getY()) < 0.01) {
                            continue;
                        }
                        if (Math.abs(tagPoses[tag2 - 1].getY()) < 0.01) {
                            continue;
                        }
                        updates.add(new TimestampVisionUpdate(timestamp,
                                new Transform3d(pose.plus(camTransforms[i].inverse()), tagPoses[tag1 - 1]),
                                getCov(offset), tag1, tag2, i));
                        break;
                    case 2: // 2 transform estimates
                        if (!enableSingleTag) {
                            continue;
                        }
                        double error1 = values[1];
                        Transform3d transform1 = new Transform3d(
                                values[2], values[3], values[4],
                                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                        double error2 = values[9];
                        Transform3d transform2 = new Transform3d(
                                values[10], values[11], values[12],
                                new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
                        int tag = (int) values[17];

                        if (Math.abs(tagPoses[tag - 1].getY()) < 0.01) {
                            continue;
                        }

                        // If the ambiguity is too high, throw out the measurement
                        if (!(error1 < error2 * 0.4 || error2 < error1 * 0.4)) {
                            continue;
                        }

                        if (tagPoses.length != 0) {
                            Rotation3d currentRotation = robotPose.getRotation();
                            Rotation3d robotPose1Rotation = tagPoses[tag - 1]
                                    .plus(camTransforms[i].plus(transform1).inverse()).getRotation();
                            Rotation3d robotPose2Rotation = tagPoses[tag - 1]
                                    .plus(camTransforms[i].plus(transform2).inverse()).getRotation();

                            if (Math.abs(currentRotation.minus(robotPose1Rotation).getAngle()) < Math
                                    .abs(currentRotation.minus(robotPose2Rotation).getAngle())) {
                                Transform3d robotToTag = camTransforms[i].plus(transform1);
                                updates.add(new TimestampVisionUpdate(timestamp,
                                        robotToTag,
                                        getCov(robotToTag),
                                        tag, 0, i));
                            } else {
                                Transform3d robotToTag = camTransforms[i].plus(transform2);
                                updates.add(new TimestampVisionUpdate(
                                        timestamp,
                                        robotToTag,
                                        getCov(robotToTag),
                                        tag, 0, i));
                            }
                        } else {
                            if (error1 < error2) {
                                Transform3d robotToTag = camTransforms[i].plus(transform1);
                                updates.add(new TimestampVisionUpdate(timestamp,
                                        robotToTag,
                                        getCov(robotToTag),
                                        tag, 0, i));
                            } else {
                                Transform3d robotToTag = camTransforms[i].plus(transform2);
                                updates.add(new TimestampVisionUpdate(timestamp,
                                        robotToTag,
                                        getCov(robotToTag),
                                        tag, 0, i));
                            }
                        }
                        break;
                }
            }
        }

        visionConsumer.accept(updates);

    }

    private SimpleMatrix getCov(Transform3d transform) {
        double val = varianceStatic + varianceScale * transform.getTranslation().getNorm();
        SimpleMatrix cov = SimpleMatrix.identity(6).scale(val);
        return cov;
    }

    public void setVisionConstants(Transform3d[] camTransforms, double varianceScale, double varianceStatic) {
        this.camTransforms = camTransforms;
        this.varianceScale = varianceScale;
        this.varianceStatic = varianceStatic;
    }

    public void setVisionConsumer(Consumer<List<TimestampVisionUpdate>> consumer) {
        this.visionConsumer = consumer;
    }

    public void setPoseSupplier(Supplier<Pose3d> supplier) {
        this.poseSupplier = supplier;
    }

    /** Represents a single vision pose */
    public static record TimestampVisionUpdate(double timestamp, Transform3d pose, SimpleMatrix covariance,
            int tagId1, int tagId2, int cameraIndex) {
    }

}
