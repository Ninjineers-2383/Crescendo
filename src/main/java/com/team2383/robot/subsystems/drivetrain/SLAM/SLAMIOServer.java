package com.team2383.robot.subsystems.drivetrain.SLAM;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import com.team2383.lib.util.TimedChassisSpeeds;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedObject;

public class SLAMIOServer implements SLAMIO {
    private final NetworkTableInstance inst;

    public final SwerveDrivePoseEstimator odometry;

    private final StructSubscriber<Pose3d> pose;
    private final DoubleSubscriber time;
    private final StructArraySubscriber<Pose3d> landmarks;
    private final StructArraySubscriber<Pose3d> seenLandmarks;

    private final StructArrayPublisher<Transform3d> camTransformsPub;
    private final DoublePublisher varianceScalePub;
    private final DoublePublisher varianceStaticPub;

    private final IntegerPublisher numLandmarksPub;
    private final StructArrayPublisher<Pose3d> landmarksPub;
    private final StructPublisher<TimedChassisSpeeds> chassisSpeedsPub;

    private long latestTimestamp = 0;

    public SLAMIOServer(SwerveDriveKinematics kinematics, SwerveModulePosition[] positions) {
        inst = NetworkTableInstance.create();
        inst.setServer(new String[] { "127.0.0.1" }, new int[] { 5811 });
        inst.startClient4("SLAM-client");

        odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), positions, new Pose2d());
        NetworkTable table = inst.getTable("slam_data");

        pose = table.getStructTopic("pose", Pose3d.struct)
                .subscribe(new Pose3d(), PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        time = table.getDoubleTopic("pose-time")
                .subscribe(0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        landmarks = table.getStructArrayTopic("landmarks", Pose3d.struct).subscribe(new Pose3d[0]);
        seenLandmarks = table.getStructArrayTopic("seenLandmarks", Pose3d.struct).subscribe(new Pose3d[0],
                PubSubOption.keepDuplicates(true));

        chassisSpeedsPub = table.getStructTopic("chassisSpeeds", TimedChassisSpeeds.struct)
                .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true), PubSubOption.periodic(0));

        camTransformsPub = table.getStructArrayTopic("camTransforms", Transform3d.struct).publish();
        varianceScalePub = table.getDoubleTopic("varianceScale").publish();
        varianceStaticPub = table.getDoubleTopic("varianceStatic").publish();

        numLandmarksPub = table.getIntegerTopic("numLandmarks").publish();
        landmarksPub = table.getStructArrayTopic("seed-landmarks", Pose3d.struct).publish();
    }

    @Override
    public void updateInputs(SLAMIOInputs inputs) {
        TimestampedObject<Pose3d> latestPose = pose.getAtomic();
        TimestampedDouble latestTime = time.getAtomic();

        if (latestPose.timestamp == latestTimestamp) {
            inputs.newValue = false;
        } else if (latestPose.timestamp > latestTimestamp) {
            odometry.addVisionMeasurement(latestPose.value.toPose2d(), latestTime.value,
                    VecBuilder.fill(0.01, 0.01, 0.01));

            inputs.connected = true;
            inputs.newValue = true;
            inputs.landmarks = landmarks.get();
            inputs.seenLandmarks = seenLandmarks.get();

            latestTimestamp = latestPose.timestamp;

            if (MathSharedStore.getTimestamp() - latestTime.value > 1) {
                inputs.connected = false;
            }
        }

        Pose2d ref = odometry.getEstimatedPosition();
        inputs.pose = new Pose3d(
                ref.getX(), ref.getY(), inputs.pose.getZ(),
                new Rotation3d(inputs.pose.getRotation().getX(),
                        inputs.pose.getRotation().getY(),
                        ref.getRotation().getRadians()));

    }

    @Override
    public void updateChassisSpeeds(ChassisSpeeds speeds, SwerveModulePosition[] modulePositions,
            Rotation2d gyroAngle) {
        chassisSpeedsPub.set(new TimedChassisSpeeds(speeds, MathSharedStore.getTimestamp()));
        odometry.update(gyroAngle, modulePositions);
    }

    @Override
    public void seedLandmarks(Pose3d[] landmarks) {
        landmarksPub.set(landmarks);
        numLandmarksPub.set(landmarks.length);
    }

    @Override
    public void setVisionConstants(Transform3d[] camPoses, double varianceScale, double varianceStatic) {
        camTransformsPub.set(camPoses);
        varianceScalePub.set(varianceScale);
        varianceStaticPub.set(varianceStatic);
    }

    @Override
    public void forcePose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        odometry.resetPosition(gyroAngle, positions, pose);
    }

    @Override
    public void forceHeading(Rotation2d heading, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        odometry.resetPosition(gyroAngle, positions,
                new Pose2d(odometry.getEstimatedPosition().getTranslation(), heading));
    }
}
