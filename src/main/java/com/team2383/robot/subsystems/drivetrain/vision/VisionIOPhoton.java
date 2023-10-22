package com.team2383.robot.subsystems.drivetrain.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionIOPhoton implements VisionIO {

    public PhotonCamera[] photonCameras = new PhotonCamera[VisionConstants.kPhotonCameras.length];
    public PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[VisionConstants.kPhotonCameras.length];
    AprilTagFieldLayout atfl;

    public VisionIOPhoton() {
        try {
            atfl = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load AprilTagFieldLayout");
        }

        for (int i = 0; i < VisionConstants.kPhotonCameras.length; i++) {
            photonCameras[i] = new PhotonCamera(VisionConstants.kPhotonCameras[i].name);
            photonPoseEstimators[i] = new PhotonPoseEstimator(
                    atfl, PoseStrategy.MULTI_TAG_PNP, photonCameras[i],
                    VisionConstants.kPhotonCameras[i].transform);
        }

        atfl.setOrigin(DriverStation.getAlliance().equals(DriverStation.Alliance.Red)
                ? OriginPosition.kRedAllianceWallRightSide
                : OriginPosition.kBlueAllianceWallRightSide);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        atfl.setOrigin(DriverStation.getAlliance().equals(DriverStation.Alliance.Red)
                ? OriginPosition.kRedAllianceWallRightSide
                : OriginPosition.kBlueAllianceWallRightSide);

        EstimatedRobotPose[] poses = new EstimatedRobotPose[photonCameras.length];
        int len = photonCameras.length;
        for (int i = 0; i < len; i++) {
            PhotonPipelineResult result = photonCameras[i].getLatestResult();
            // inputs.tagsSeen[i] = new int[result.targets.size()];
            double minAmbiguity = 0;
            for (int j = 0; j < result.targets.size(); j++) {
                double ambiguity = result.targets.get(j).getPoseAmbiguity();
                if (minAmbiguity > ambiguity) {
                    minAmbiguity = ambiguity;
                }
                // inputs.tagsSeen[i][j] = result.targets.get(j).getFiducialId();
            }

            if (minAmbiguity > 0.4) {
                continue;
            }

            // photonPoseEstimators[i].setReferencePose(prevEstimatedRobotPose);
            Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimators[i].update();
            if (estimatedRobotPose.isEmpty()) {
                continue;
            }
            poses[i] = estimatedRobotPose.get();
        }

        inputs.clear(len);

        for (int i = 0; i < len; i++) {
            if (poses[i] == null) {
                inputs.connected[i] = false;
                continue;
            }
            setPose(inputs, poses[i], i);
        }
    }

    private void setPose(VisionIOInputs inputs, EstimatedRobotPose pose, int i) {
        inputs.connected[i] = true;
        inputs.x[i] = pose.estimatedPose.getX();
        inputs.y[i] = pose.estimatedPose.getY();
        inputs.z[i] = pose.estimatedPose.getZ();
        inputs.roll[i] = pose.estimatedPose.getRotation().getX();
        inputs.pitch[i] = pose.estimatedPose.getRotation().getY();
        inputs.yaw[i] = pose.estimatedPose.getRotation().getZ();
        inputs.timestampSeconds[i] = pose.timestampSeconds;
    }
}
