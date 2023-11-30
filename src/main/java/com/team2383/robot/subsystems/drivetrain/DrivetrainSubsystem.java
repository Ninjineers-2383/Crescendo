package com.team2383.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.AllianceUtil;
import com.pathplanner.lib.util.FieldMirroring.MirroringType;
import com.pathplanner.lib.util.FieldMirroring.Origin;
import com.team2383.robot.helpers.LocalADStarAK;
import com.team2383.robot.subsystems.vision.VisionSubsystem.TimestampVisionUpdate;
import com.team2383.lib.SLAM.EKFSLAM;

import java.util.List;

public class DrivetrainSubsystem extends SubsystemBase {
    private final CoaxialSwerveModule m_frontLeftModule;
    private final CoaxialSwerveModule m_frontRightModule;
    private final CoaxialSwerveModule m_rearLeftModule;
    private final CoaxialSwerveModule m_rearRightModule;

    private final CoaxialSwerveModule[] m_modules;
    private final SwerveModuleState[] m_lastStates;

    private final GyroIO m_gyro;
    private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();

    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            DriveConstants.frontLeftConstants.translation,
            DriveConstants.frontRightConstants.translation,
            DriveConstants.rearLeftConstants.translation,
            DriveConstants.rearRightConstants.translation);

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final EKFSLAM m_slam;

    private final Field2d m_field = new Field2d();
    private final FieldObject2d m_COR;

    private int loop_cycle = 0;

    private double headingIntegral = 0;

    private ChassisSpeeds m_robotRelativeChassisSpeeds = new ChassisSpeeds();
    private AprilTagFieldLayout aprilTags;

    public DrivetrainSubsystem(GyroIO gyro, SwerveModuleIO frontLeftIO, SwerveModuleIO frontRightIO,
            SwerveModuleIO rearLeftIO, SwerveModuleIO rearRightIO) {
        m_gyro = gyro;

        m_frontLeftModule = new CoaxialSwerveModule(frontLeftIO, "FL");
        m_frontRightModule = new CoaxialSwerveModule(frontRightIO, "FR");
        m_rearLeftModule = new CoaxialSwerveModule(rearLeftIO, "RL");
        m_rearRightModule = new CoaxialSwerveModule(rearRightIO, "RR");

        m_modules = new CoaxialSwerveModule[] { m_frontLeftModule, m_frontRightModule, m_rearLeftModule,
                m_rearRightModule };

        m_poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                new Rotation2d(),
                getModulePositions(),
                new Pose2d());

        m_slam = new EKFSLAM(8);
        try {
            aprilTags = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            aprilTags = new AprilTagFieldLayout(null, 0, 0);
        }
        aprilTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        m_lastStates = new SwerveModuleState[m_modules.length];

        SmartDashboard.putData("Field", m_field);
        m_COR = m_field.getObject("COR");

        addChild(DriveConstants.frontLeftConstants.name, m_frontLeftModule);
        addChild(DriveConstants.frontRightConstants.name, m_frontRightModule);
        addChild(DriveConstants.rearLeftConstants.name, m_rearLeftModule);
        addChild(DriveConstants.rearRightConstants.name, m_rearRightModule);

        if (RobotBase.isSimulation()) {
            m_poseEstimator.resetPosition(new Rotation2d(), getModulePositions(),
                    new Pose2d(new Translation2d(0, 0), new Rotation2d()));
        }

        resetHeading();

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::forceOdometry,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                DriveConstants.CONFIG,
                MirroringType.HORIZONTAL,
                Origin.BLUE,
                new LocalADStarAK(MirroringType.HORIZONTAL, Origin.BLUE),
                this);

        Pose3d[] landmarks = new Pose3d[8];

        for (int i = 1; i <= 8; i++) {
            landmarks[i - 1] = aprilTags.getTagPose(i).get();
        }

        m_slam.seedLandmarks(landmarks);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            for (CoaxialSwerveModule module : m_modules) {
                module.stop();
            }
        }

        if (loop_cycle == 200) {
            for (CoaxialSwerveModule module : m_modules) {
                module.resetToAbsolute();
            }
        }

        loop_cycle++;

        m_gyro.updateInputs(m_gyroInputs);
        Logger.processInputs("Gyro", m_gyroInputs);

        for (CoaxialSwerveModule module : m_modules) {
            module.periodic();
        }

        for (int i = 0; i < m_modules.length; i++) {
            m_lastStates[i] = m_modules[i].getState();
        }

        ChassisSpeeds chassis = m_kinematics.toChassisSpeeds(m_lastStates);

        headingIntegral += chassis.omegaRadiansPerSecond * 0.02;

        if (m_gyroInputs.connected) {
            m_poseEstimator.update(Rotation2d.fromDegrees(m_gyroInputs.headingDeg), getModulePositions());
        } else {
            m_poseEstimator.update(
                    Rotation2d.fromRadians(headingIntegral),
                    getModulePositions());
        }

        m_slam.predict(chassis, 0.02);

        Pose2d estimatedPose = m_poseEstimator.getEstimatedPosition();

        m_field.setRobotPose(estimatedPose);

        SmartDashboard.putNumber("Roll", getRoll());

        Logger.recordOutput("Swerve/Real Module States", m_lastStates);

        Logger.recordOutput("Swerve/Heading Integral", headingIntegral);

        Logger.recordOutput("Swerve/Chassis Heading", chassis.omegaRadiansPerSecond);

        Logger.recordOutput("Robot Pose", estimatedPose);

        Pose3d slamPose3d = m_slam.getRobotPose();

        Logger.recordOutput("SLAM/Pose", slamPose3d);

        Pose3d[] landmarks = m_slam.getLandmarkPoses();

        Logger.recordOutput("SLAM/Landmarks", landmarks);
    }

    /**
     * Drive the robot using field or robot relative velocity
     * 
     * @param drive
     *            The speed for driving
     * @param angle
     *            The set angular speed
     * @param fieldRelative
     *            Whether the speeds are relative to the field or the
     *            robot
     * @param centerOfRotation
     *            Its the center of rotation duh
     */
    public void drive(Translation2d drive, Rotation2d angle, boolean fieldRelative,
            Translation2d centerOfRotation) {
        ChassisSpeeds speeds;

        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(drive.getX(), drive.getY(), angle.getRadians(),
                    getHeading());
        } else {
            speeds = new ChassisSpeeds(drive.getX(), drive.getY(), angle.getRadians());
        }

        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                speeds,
                centerOfRotation);

        setModuleStates(swerveModuleStates);

        Logger.recordOutput("Swerve/Desired Module States", swerveModuleStates);

        m_COR.setPose(getPose().plus(new Transform2d(centerOfRotation, new Rotation2d())));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_robotRelativeChassisSpeeds;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        m_robotRelativeChassisSpeeds = speeds;

        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                speeds);

        setModuleStates(swerveModuleStates);

        Logger.recordOutput("Swerve/Desired Module States", swerveModuleStates);

        m_COR.setPose(getPose());
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeed);

        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setDesiredState(states[i]);
        }
    }

    public void resetEncoders() {
        // for (CoaxialSwerveModule module : m_modules) {
        // module.resetEncoders();
        // }
    }

    public void visionConsumer(List<TimestampVisionUpdate> visionUpdates) {
        Pose3d pose = getPose3d();

        Pose3d[] landmarks = new Pose3d[visionUpdates.size()];
        int i = 0;
        for (TimestampVisionUpdate update : visionUpdates) {
            if (!m_slam.isEnabled()) {
                m_slam.setInitialRobotPose(aprilTags.getTagPose(update.tagId()).get().plus(update.pose().inverse()));
            }
            Pose3d tagPose = pose.plus(update.pose());
            landmarks[i] = tagPose;
            m_slam.correct(update.pose(), update.tagId() - 1);
            i++;
        }

        Logger.recordOutput("Vision/Targets", landmarks);
    }

    /**
     * Get the current robot heading
     * Note: This is normalized so that 0 is facing directly away from your alliance
     * wall
     * <p>
     * Note: CCW is positive
     * 
     * @return The heading of the robot in a Rotation2D
     */
    public Rotation2d getHeading() {
        return m_poseEstimator.getEstimatedPosition().getRotation();
    }

    /**
     * Reset the heading to the param
     * 
     * @param currentHeading
     *            the heading to reset the gyro to. This must be in field
     *            relative coordinates when CCW is position and 0 is
     *            facing directly towards the opposing alliance wall
     */
    public void forceHeading(Rotation2d currentHeading) {
        // m_gyro.setHeading(currentHeading);
        m_poseEstimator.resetPosition(currentHeading, getModulePositions(), m_poseEstimator.getEstimatedPosition());
    }

    /**
     * Set the current heading to the calculated compass heading
     * <p>
     * Gyro offset needs to be saved to robot before this can be used.
     * If the compass heading is not stored this will set the forward direction to
     * north
     */
    public void resetHeading() {
        m_gyro.setHeading(new Rotation2d());
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(),
                new Pose2d(m_poseEstimator.getEstimatedPosition().getTranslation(), getHeading()));
        resetEncoders();
    }

    /**
     * Set the current heading to the calculated compass heading
     * <p>
     * Gyro offset needs to be saved to robot before this can be used.
     * If the compass heading is not stored this will set the forward direction to
     * north
     */
    public void resetHeading(Rotation2d offset) {
        m_gyro.setHeading(offset);
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(),
                new Pose2d(m_poseEstimator.getEstimatedPosition().getTranslation(), getHeading()));
        resetEncoders();
    }

    /**
     * Get turn rate of robot
     * 
     * @return turn rate in degrees per second CCW positive
     */
    public double getTurnRate() {
        return m_gyroInputs.headingRateDPS;
    }

    public double getRoll() {
        return m_gyroInputs.rollDeg;
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public Pose3d getEstimatorPose3d() {
        return new Pose3d(getPose());
    }

    public Pose3d getPose3d() {
        return m_slam.getRobotPose();
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                m_rearLeftModule.getPosition(), m_rearRightModule.getPosition() };
    }

    public void forceOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
        resetEncoders();
    }

    public void forceOdometryAlliance(Pose2d pose) {
        pose = AllianceUtil.transformPoseForAlliance(pose, AutoBuilder.getMirroringType(), AutoBuilder.getOrigin());

        forceOdometry(pose);
    }

    public void setPosition(Translation2d position) {
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(), new Pose2d(position, getHeading()));
        resetEncoders();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}