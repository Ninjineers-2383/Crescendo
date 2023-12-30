package com.team2383.robot.subsystems.drivetrain.SLAM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedObject;

public class SLAMIOServer implements SLAMIO {
    public final SwerveDriveOdometry odometry;

    private final StructSubscriber<Pose3d> pose;
    private final StructArraySubscriber<Pose3d> landmarks;
    private final StructArraySubscriber<Pose3d> seenLandmarks;

    private final IntegerPublisher numLandmarksPub;
    private final StructArrayPublisher<Pose3d> landmarksPub;
    private final StructPublisher<ChassisSpeeds> chassisSpeedsPub;

    private long latestTimestamp = 0;

    public SLAMIOServer(SwerveDriveKinematics kinematics, SwerveModulePosition[] positions) {
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), positions);
        NetworkTable table = NetworkTableInstance.getDefault().getTable("slam_data");

        pose = table.getStructTopic("pose", Pose3d.struct)
                .subscribe(new Pose3d(), PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        landmarks = table.getStructArrayTopic("landmarks", Pose3d.struct).subscribe(new Pose3d[0]);
        seenLandmarks = table.getStructArrayTopic("seenLandmarks", Pose3d.struct).subscribe(new Pose3d[0]);

        chassisSpeedsPub = table.getStructTopic("chassisSpeeds", ChassisSpeeds.struct)
                .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true), PubSubOption.periodic(0));
        numLandmarksPub = table.getIntegerTopic("numLandmarks").publish();
        landmarksPub = table.getStructArrayTopic("seed-landmarks", Pose3d.struct).publish();
    }

    @Override
    public void updateInputs(SLAMIOInputs inputs) {
        TimestampedObject<Pose3d> latestPose = pose.getAtomic();

        if (latestPose.timestamp == latestTimestamp) {
            inputs.newValue = false;
            Pose2d ref = odometry.getPoseMeters();
            inputs.pose = new Pose3d(
                    ref.getX(), ref.getY(), inputs.pose.getZ(),
                    new Rotation3d(inputs.pose.getRotation().getX(), inputs.pose.getRotation().getY(),
                            ref.getRotation().getRadians()));
            return;
        }

        inputs.connected = true;
        inputs.newValue = true;
        inputs.pose = latestPose.value;
        inputs.landmarks = landmarks.get();
        inputs.seenLandmarks = seenLandmarks.get();

        latestTimestamp = latestPose.timestamp;
    }

    @Override
    public void updateChassisSpeeds(ChassisSpeeds speeds, SwerveModulePosition[] modulePositions,
            Rotation2d gyroAngle) {
        chassisSpeedsPub.set(speeds);
        odometry.update(gyroAngle, modulePositions);
    }

    @Override
    public void seedLandmarks(Pose3d[] landmarks) {
        landmarksPub.set(landmarks);
        numLandmarksPub.set(landmarks.length);
    }

    @Override
    public void forcePose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        odometry.resetPosition(gyroAngle, positions, pose);
    }

    @Override
    public void forceHeading(Rotation2d heading, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        odometry.resetPosition(gyroAngle, positions, new Pose2d(odometry.getPoseMeters().getTranslation(), heading));
    }
}
