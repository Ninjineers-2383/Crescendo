package com.team2383.robot.subsystems.drivetrain.SLAM;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedObject;

public class SLAMIOServer implements SLAMIO {
    private final NetworkTableInstance inst;

    private final SwerveDrivePoseEstimator odometry;
    private final SwerveDriveKinematics kinematics;

    private final StructSubscriber<Pose3d> pose;
    private final DoubleSubscriber time;
    private final BooleanPublisher saveAndExit;

    private final StructArrayPublisher<Transform3d> camTransformsPub;
    private final DoublePublisher varianceScalePub;
    private final DoublePublisher varianceStaticPub;

    private final StructPublisher<Twist3d> twist3dPub;

    private long latestTimestamp = 0;

    private SwerveDriveWheelPositions lastPositions;

    public SLAMIOServer(SwerveDriveKinematics kinematics, SwerveModulePosition[] positions) {
        inst = NetworkTableInstance.create();
        inst.setServer(new String[] { "127.0.0.1", "10.23.83.11" }, new int[] { 5811, 5810 });
        inst.startClient4("SLAM-client");

        odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), positions, new Pose2d());
        this.kinematics = kinematics;
        NetworkTable table = inst.getTable("slam_data");
        pose = table.getStructTopic("pose", Pose3d.struct)
                .subscribe(new Pose3d(), PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        time = table.getDoubleTopic("pose-time")
                .subscribe(0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

        twist3dPub = table.getStructTopic("twist3d", Twist3d.struct)
                .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true), PubSubOption.periodic(0));

        saveAndExit = table.getBooleanTopic("saveAndExit").publish();

        camTransformsPub = table.getStructArrayTopic("camTransforms", Transform3d.struct).publish();
        varianceScalePub = table.getDoubleTopic("varianceScale").publish();
        varianceStaticPub = table.getDoubleTopic("varianceStatic").publish();

        lastPositions = new SwerveDriveWheelPositions(positions);
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
    public void updateModulePositions(SwerveModulePosition[] modulePositions,
            Rotation2d gyroAngle) {
        SwerveDriveWheelPositions positions = new SwerveDriveWheelPositions(modulePositions);
        Twist2d twist = kinematics.toTwist2d(lastPositions, positions);
        twist3dPub.set(new Twist3d(twist.dx, twist.dy, 0, 0, 0, twist.dtheta));
        odometry.update(gyroAngle, modulePositions);

        lastPositions = positions;
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

    @Override
    public void saveAndExit(boolean saveAndExit) {
        this.saveAndExit.set(saveAndExit);
    }
}
