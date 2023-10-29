package com.team2383.robot.subsystems.drivetrain.vision;

import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;

public class VisionIOSim implements VisionIO {
    AprilTagFieldLayout atfl;

    public VisionIOSim() {
        try {
            atfl = new AprilTagFieldLayout(
                    Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "2023-chargedup-shift.json"));
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load AprilTagFieldLayout");
        }

        atfl.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
        inputs.clear(4);

        int cam = 0;

        for (AprilTag tag : atfl.getTags()) {
            Transform3d robotToTag = new Transform3d(robotPose, atfl.getTagPose(tag.ID).get());
            if (robotToTag.getTranslation().getNorm() < 4) {

                inputs.connected[cam] = true;
                inputs.x[cam] = robotToTag.getTranslation().getX();
                inputs.y[cam] = robotToTag.getTranslation().getY();
                inputs.z[cam] = robotToTag.getTranslation().getZ();

                inputs.rw[cam] = robotToTag.getRotation().getQuaternion().getW();
                inputs.rx[cam] = robotToTag.getRotation().getQuaternion().getX();
                inputs.ry[cam] = robotToTag.getRotation().getQuaternion().getY();
                inputs.rz[cam] = robotToTag.getRotation().getQuaternion().getZ();

                inputs.tagId[cam] = tag.ID;

                inputs.ambiguity[cam] = robotToTag.getTranslation().getNorm() / 4;

                inputs.timestampSeconds[cam] = RobotController.getFPGATime() / 1e6;

                cam++;

                if (cam >= inputs.connected.length) {
                    break;
                }
            }
        }
    }
}
