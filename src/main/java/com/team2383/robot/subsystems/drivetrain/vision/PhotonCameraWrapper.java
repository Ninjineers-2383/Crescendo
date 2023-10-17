package com.team2383.robot.subsystems.drivetrain.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimVisionSystem;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.team2383.robot.Robot;

public class PhotonCameraWrapper {
    private int numCameras = Robot.isReal() ? VisionConstants.kPhotonCameras.length : 1;
    public PhotonCamera[] photonCameras = new PhotonCamera[numCameras];
    public PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[numCameras];
    public SimVisionSystem simVisionSystem = null;
    private AprilTagFieldLayout atfl;

    public PhotonCameraWrapper() {
        try {
            atfl = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load AprilTagFieldLayout");
        }

        for (int i = 0; i < numCameras; i++) {
            photonCameras[i] = new PhotonCamera(VisionConstants.kPhotonCameras[i].name);
            photonPoseEstimators[i] = new PhotonPoseEstimator(
                    atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCameras[i],
                    VisionConstants.kPhotonCameras[i].transform);
        }

        atfl.setOrigin(DriverStation.getAlliance().equals(DriverStation.Alliance.Red)
                ? OriginPosition.kRedAllianceWallRightSide
                : OriginPosition.kBlueAllianceWallRightSide);

        if (Robot.isSimulation()) {
            simVisionSystem = new SimVisionSystem(
                    VisionConstants.kPhotonCameras[0].name,
                    100,
                    VisionConstants.kPhotonCameras[0].transform,
                    9000,
                    1280,
                    800,
                    0.0);

            simVisionSystem.addVisionTargets(atfl);
        }
    }

    public void simulate(Pose2d robotPose) {
        if (Robot.isSimulation()) {
            simVisionSystem.processFrame(robotPose);
        }
    }

    /**
     * @param estimatedRobotPose
     *            The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the
     *         field, and the time
     *         of the observation. Assumes a planar field and the robot is always
     *         firmly on the ground
     */
    public EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        atfl.setOrigin(DriverStation.getAlliance().equals(DriverStation.Alliance.Red)
                ? OriginPosition.kRedAllianceWallRightSide
                : OriginPosition.kBlueAllianceWallRightSide);

        EstimatedRobotPose latestPose = null;
        int len = Robot.isReal() ? photonCameras.length : 1;
        for (int i = 0; i < len; i++) {
            PhotonPipelineResult result = photonCameras[i].getLatestResult();
            double minAmbiguity = 0;
            for (PhotonTrackedTarget target : result.targets) {
                double ambiguity = target.getPoseAmbiguity();
                if (minAmbiguity > ambiguity) {
                    minAmbiguity = ambiguity;
                }
            }

            if (minAmbiguity > 0.4)
                continue;

            photonPoseEstimators[i].setReferencePose(prevEstimatedRobotPose);
            Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimators[i].update();
            if (estimatedRobotPose.isEmpty()) {
                continue;
            }
            if (latestPose == null || estimatedRobotPose.get().timestampSeconds > latestPose.timestampSeconds) {
                latestPose = estimatedRobotPose.get();
            }
        }

        return latestPose;
    }
}