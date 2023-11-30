package com.team2383.robot.subsystems.vision;

import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;

import java.util.ArrayList;
import java.util.Random;

public class VisionIOSim implements VisionIO {
    AprilTagFieldLayout atfl;
    Transform3d cameraTransform;

    public VisionIOSim(Transform3d cameraTransform) {
        try {
            atfl = new AprilTagFieldLayout(
                    Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "2023-chargedup-shift.json"));
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load AprilTagFieldLayout");
        }

        atfl.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        this.cameraTransform = cameraTransform;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
        Pose3d cameraPose = robotPose.plus(cameraTransform);
        ArrayList<AprilTag> tags = new ArrayList<>();
        for (AprilTag tag : atfl.getTags()) {
            Transform3d robotToTag = new Transform3d(cameraPose, atfl.getTagPose(tag.ID).get());
            if (robotToTag.getTranslation().getNorm() < 4) {
                tags.add(tag);
            }
        }
        if (tags.size() > 0) {
            AprilTag tag = tags.get(new Random().nextInt(tags.size()));
            Transform3d robotToTag = new Transform3d(cameraPose, atfl.getTagPose(tag.ID).get());
            inputs.timestamps = new double[] { RobotController.getFPGATime() / 1000000.0 };
            inputs.frames = new double[][] { new double[] {
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
            } };
            inputs.fps = 24;
        } else {
            inputs.timestamps = new double[0];
            inputs.frames = new double[0][];
            inputs.fps = 0;
        }
    }

}
