package com.team2383.robot.subsystems.drivetrain.SLAM;

import com.team2383.lib.CameraParameters;
import com.team2383.lib.util.TimedPose3d;
import com.team2383.lib.util.TimedRobotUpdate;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedObject;

public class SLAMIOServer implements SLAMIO {
    private final NetworkTableInstance inst;

    private final SwerveDrivePoseEstimator odometry;

    private final StructPublisher<TimedRobotUpdate> robotPub;
    private final StructArrayPublisher<Translation2d> moduleLocationsPub;
    private final StructArrayPublisher<Pose3d> landmarksPub;

    private final StructSubscriber<TimedPose3d> pose;

    private final BooleanPublisher saveAndExit;

    private final StructArrayPublisher<Transform3d> camTransformsPub;
    private final DoublePublisher varianceScalePub;
    private final DoublePublisher varianceStaticPub;
    private final IntegerPublisher poseFilterSizePub;
    private final DoublePublisher poseOutlierRejectionDistanceSub;
    private final StructPublisher<Pose2d> resetPose;
    private final StructPublisher<CameraParameters> cameraParameters;

    private double latestTimestamp = 0;

    public SLAMIOServer(Translation2d[] moduleLocations, Pose3d[] landmarks) {
        inst = NetworkTableInstance.create();
        inst.setServer(new String[] { "127.0.0.1", "10.23.83.11" }, new int[] { 5811, 5810 });
        inst.startClient4("SLAM-client");

        odometry = new SwerveDrivePoseEstimator(new SwerveDriveKinematics(moduleLocations), new Rotation2d(),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                },
                new Pose2d());
        NetworkTable table = inst.getTable("slam_data");

        robotPub = table.getStructTopic("robotUpdate", TimedRobotUpdate.struct).publish(PubSubOption.periodic(0),
                PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

        moduleLocationsPub = table.getStructArrayTopic("moduleLocations", Translation2d.struct).publish();

        landmarksPub = table.getStructArrayTopic("landmarks", Pose3d.struct).publish(PubSubOption.sendAll(true),
                PubSubOption.keepDuplicates(true));

        pose = table.getStructTopic("pose", TimedPose3d.struct).subscribe(new TimedPose3d());

        saveAndExit = table.getBooleanTopic("saveAndExit").publish();

        resetPose = table.getStructTopic("PoseReset", Pose2d.struct).publish();

        camTransformsPub = table.getStructArrayTopic("camTransforms", Transform3d.struct).publish();
        varianceScalePub = table.getDoubleTopic("varianceScale").publish();
        varianceStaticPub = table.getDoubleTopic("varianceStatic").publish();

        poseFilterSizePub = table.getIntegerTopic("poseFilterSize").publish();
        poseOutlierRejectionDistanceSub = table.getDoubleTopic("PoseOutlierDistance").publish();

        cameraParameters = table.getStructTopic("camParameters", CameraParameters.struct).publish();

        moduleLocationsPub.set(moduleLocations);
        landmarksPub.set(landmarks);
    }

    @Override
    public void updateInputs(SLAMIOInputs inputs) {
        TimestampedObject<TimedPose3d> latestPose = pose.getAtomic();

        if (latestPose.value.timestamp <= latestTimestamp) {
            inputs.newValue = false;
        } else {
            inputs.latestTimestamp = latestPose.value.timestamp;
            odometry.addVisionMeasurement(latestPose.value.pose.toPose2d(), latestPose.value.timestamp,
                    VecBuilder.fill(0.01, 0.01, 0.01));

            inputs.connected = true;
            inputs.newValue = true;

            if (MathSharedStore.getTimestamp() - latestPose.value.timestamp > 1) {
                inputs.connected = false;
            }
            latestTimestamp = latestPose.value.timestamp;
        }

        Pose2d ref = odometry.getEstimatedPosition();
        inputs.pose = new Pose3d(
                ref.getX(), ref.getY(), 0,
                new Rotation3d(0, 0,
                        ref.getRotation().getRadians()));
        inputs.seenLandmarks = latestPose.value.seenTags;

    }

    @Override
    public void updateModulePositions(SwerveModulePosition[] modulePositions,
            Rotation2d gyroAngle) {
        Rotation3d gyroAngle3d = new Rotation3d(0, 0, gyroAngle.getRadians());
        odometry.update(gyroAngle, modulePositions);

        robotPub.set(new TimedRobotUpdate(modulePositions, gyroAngle3d, MathSharedStore.getTimestamp()));
    }

    @Override
    public void setVisionConstants(Transform3d[] camPoses, double varianceScale, double varianceStatic,
            CameraParameters camParams) {
        camTransformsPub.set(camPoses);
        varianceScalePub.set(varianceScale);
        varianceStaticPub.set(varianceStatic);

        poseFilterSizePub.set(15);
        poseOutlierRejectionDistanceSub.set(0.1);
        cameraParameters.set(camParams);
    }

    @Override
    public void forcePose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        latestTimestamp = MathSharedStore.getTimestamp() + 1;
        odometry.resetPosition(gyroAngle, positions, pose);
        resetPose.set(pose);
    }

    @Override
    public void forceHeading(Rotation2d heading, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        forcePose(new Pose2d(odometry.getEstimatedPosition().getTranslation(), heading), gyroAngle, positions);
    }

    @Override
    public void saveAndExit(boolean saveAndExit) {
        this.saveAndExit.set(saveAndExit);
    }
}
