package com.team2383.robot.subsystems.cameraSim;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.nio.file.Path;
import java.util.function.Supplier;

import com.team2383.robot.helpers.Noise;

import java.util.ArrayList;
import java.util.Random;

public class CameraSimSubsystem extends SubsystemBase {
    private final NetworkTableInstance inst = NetworkTableInstance.create();
    private final AprilTagFieldLayout atfl;
    private final Transform3d cameraTransform;

    private final Supplier<Pose3d> poseSupplier;

    private final IntegerPublisher fpsPub;
    private final DoubleArrayPublisher observationsPub;

    public CameraSimSubsystem(String camera_id, Transform3d cameraTransform, Supplier<Pose3d> poseSupplier) {
        inst.setServer(new String[] { "127.0.0.1", "10.23.83.11" }, new int[] { 5811, 5810 });
        inst.startClient4("camera-sim");

        try {
            atfl = new AprilTagFieldLayout(
                    Path.of(Filesystem.getDeployDirectory().getAbsolutePath(),
                            "2024-crescendo.json"));
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load AprilTagFieldLayout");
        }

        atfl.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        this.cameraTransform = cameraTransform;
        this.poseSupplier = poseSupplier;

        NetworkTable table = inst.getTable("/" + camera_id + "/output");

        fpsPub = table.getIntegerTopic("fps").publish();
        observationsPub = table.getDoubleArrayTopic("observations").publish(PubSubOption.periodic(0),
                PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

    }

    @Override
    public void periodic() {
        Pose3d cameraPose = poseSupplier.get().plus(cameraTransform);
        ArrayList<AprilTag> tags = new ArrayList<>();
        for (AprilTag tag : atfl.getTags()) {
            Transform3d robotToTag = new Transform3d(cameraPose, atfl.getTagPose(tag.ID).get());
            if (robotToTag.getTranslation().getNorm() < 4) {
                tags.add(tag);
            }
        }
        if (tags.size() == 0)
            return;

        AprilTag tag = tags.get(new Random().nextInt(tags.size()));
        Transform3d robotToTag = new Transform3d(cameraPose, atfl.getTagPose(tag.ID).get());
        Transform3d noisyTransform = Noise.noisyTransform(0, 0.1);
        robotToTag = robotToTag.plus(noisyTransform);
        double[] observation = new double[] {

                2, // 2 pose estimates
                0, // 0 error
                robotToTag.getTranslation().getX(),
                robotToTag.getTranslation().getY(),
                robotToTag.getTranslation().getZ(),
                robotToTag.getRotation().getQuaternion().getW(),
                robotToTag.getRotation().getQuaternion().getX(),
                robotToTag.getRotation().getQuaternion().getY(),
                robotToTag.getRotation().getQuaternion().getZ(),
                1, // 1 error
                0, 0, 0, 0, 0, 0, 0,
                tag.ID
        };
        long fps = 50;

        observationsPub.set(observation);
        fpsPub.set(fps);
    }
}
