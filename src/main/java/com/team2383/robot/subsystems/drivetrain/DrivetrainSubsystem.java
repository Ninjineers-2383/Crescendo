package com.team2383.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team2383.robot.subsystems.drivetrain.SLAM.SLAMClient;
import com.team2383.robot.subsystems.drivetrain.SLAM.SLAMConstantsConfig;
import com.team2383.robot.subsystems.drivetrain.SLAM.SLAMIOServer;
import com.team2383.robot.subsystems.drivetrain.SLAM.SLAMUpdate;

public class DrivetrainSubsystem extends SubsystemBase {
    // Swerve Module Initialization
    private final CoaxialSwerveModule m_frontLeftModule;
    private final CoaxialSwerveModule m_frontRightModule;
    private final CoaxialSwerveModule m_rearLeftModule;
    private final CoaxialSwerveModule m_rearRightModule;

    private final CoaxialSwerveModule[] m_modules;
    private final SwerveModuleState[] m_lastStates;

    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            DriveConstants.frontLeftConstants.translation,
            DriveConstants.frontRightConstants.translation,
            DriveConstants.rearLeftConstants.translation,
            DriveConstants.rearRightConstants.translation);

    // Odometry Initialization
    private final SwerveDriveOdometry m_deadReckoning;
    private double headingIntegral = 0;

    // Gyro Initialization
    private final GyroIO m_gyro;
    private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();

    // SLAM Initialization
    private final SLAMClient m_SLAMClient;
    private AprilTagFieldLayout aprilTags;
    private Pose3d m_slamRobotPose = new Pose3d();

    // Field Sim Initialization
    private final Field2d m_field = new Field2d();
    private final FieldObject2d m_COR;

    // Loop cycle counter for absolute encoder initialization
    private int loop_cycle = 0;

    // Robot Control Input Robot Relative
    private ChassisSpeeds m_robotRelativeChassisSpeeds = new ChassisSpeeds();

    // Heading Controller Initialization
    private final PIDController m_headingController = DriveConstants.HEADING_CONTROLLER;
    private boolean m_headingControllerEnabled = false;
    private Rotation2d setHeading = new Rotation2d();

    public DrivetrainSubsystem(GyroIO gyro, SwerveModuleIO frontLeftIO, SwerveModuleIO frontRightIO,
            SwerveModuleIO rearLeftIO, SwerveModuleIO rearRightIO) {

        m_frontLeftModule = new CoaxialSwerveModule(frontLeftIO, "FL");
        m_frontRightModule = new CoaxialSwerveModule(frontRightIO, "FR");
        m_rearLeftModule = new CoaxialSwerveModule(rearLeftIO, "RL");
        m_rearRightModule = new CoaxialSwerveModule(rearRightIO, "RR");

        m_modules = new CoaxialSwerveModule[] { m_frontLeftModule, m_frontRightModule, m_rearLeftModule,
                m_rearRightModule };
        m_lastStates = new SwerveModuleState[m_modules.length];

        m_deadReckoning = new SwerveDriveOdometry(m_kinematics, getHeading(), getModulePositions());

        m_gyro = gyro;

        m_SLAMClient = new SLAMClient(new SLAMIOServer(m_kinematics, getModulePositions()));

        try {
            aprilTags = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (Exception e) {
            aprilTags = new AprilTagFieldLayout(null, 0, 0);
        }

        initializeSLAM();

        SmartDashboard.putData("Field", m_field);
        m_COR = m_field.getObject("COR");

        m_headingController.enableContinuousInput(0, 2 * Math.PI);

        forceHeading(new Rotation2d());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::forceOdometry,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                DriveConstants.CONFIG,
                () -> true,
                this);

        addChild(DriveConstants.frontLeftConstants.name, m_frontLeftModule);
        addChild(DriveConstants.frontRightConstants.name, m_frontRightModule);
        addChild(DriveConstants.rearLeftConstants.name, m_rearLeftModule);
        addChild(DriveConstants.rearRightConstants.name, m_rearRightModule);
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

        m_robotRelativeChassisSpeeds = m_kinematics.toChassisSpeeds(m_lastStates);

        headingIntegral += m_robotRelativeChassisSpeeds.omegaRadiansPerSecond * 0.02;
        headingIntegral %= (Math.PI);

        SLAMUpdate update;
        if (m_gyroInputs.connected) {
            update = m_SLAMClient.update(getModulePositions(),
                    Rotation2d.fromDegrees(m_gyroInputs.headingDeg));
            m_deadReckoning.update(Rotation2d.fromDegrees(m_gyroInputs.headingDeg), getModulePositions());
        } else {
            update = m_SLAMClient.update(getModulePositions(),
                    Rotation2d.fromRadians(headingIntegral));
            m_deadReckoning.update(Rotation2d.fromRadians(headingIntegral), getModulePositions());
        }

        if (m_headingControllerEnabled) {
            double output = m_headingController.calculate(getHeading().getRadians(), setHeading.getRadians());
            driveRobotRelative(new ChassisSpeeds(0, 0, output));
        }

        m_slamRobotPose = update.pose();

        m_field.setRobotPose(update.pose().toPose2d());

        SmartDashboard.putNumber("Roll", getRoll());

        Logger.recordOutput("Swerve/Real Module States", m_lastStates);

        Logger.recordOutput("Swerve/Chassis Heading", headingIntegral);

        Logger.recordOutput("Swerve/Chassis Heading Velocity", m_robotRelativeChassisSpeeds.omegaRadiansPerSecond);

        Logger.recordOutput("Robot Pose", update.pose().toPose2d());

        Logger.recordOutput("SLAM/Robot Pose", update.pose());
        Logger.recordOutput("SLAM/landmarks", update.landmarks());
        Logger.recordOutput("SLAM/seenLandmarks", update.seenLandmarks());
        Logger.recordOutput("SLAM/newValue", update.newValue());

        Logger.recordOutput("Swerve/Dead Reckoning", m_deadReckoning.getPoseMeters());
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

        if (angle.getRadians() != 0) {
            m_headingControllerEnabled = false;
            setHeading = getHeading();
        } else {
            m_headingControllerEnabled = true;
        }

        if (fieldRelative) {
            m_robotRelativeChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(drive.getX(), drive.getY(),
                    angle.getRadians(),
                    getHeading());
        } else {
            m_robotRelativeChassisSpeeds = new ChassisSpeeds(drive.getX(), drive.getY(), angle.getRadians());
        }

        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                m_robotRelativeChassisSpeeds,
                centerOfRotation);

        setModuleStates(swerveModuleStates);

        Logger.recordOutput("Swerve/Desired Module States", swerveModuleStates);

        m_COR.setPose(getPose().plus(new Transform2d(centerOfRotation, new Rotation2d())));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_robotRelativeChassisSpeeds;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(m_robotRelativeChassisSpeeds, getHeading());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        m_headingControllerEnabled = false;

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
        // return Rotation2d.fromRadians(headingIntegral);
        return Rotation2d.fromRadians(m_slamRobotPose.getRotation().getZ());
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
        m_SLAMClient.forceHeading(currentHeading, new Rotation2d(m_gyroInputs.headingDeg), getModulePositions());
    }

    /**
     * Force the current odometry pose of the robot to the pose passed into the
     * parameter. Will not permanently change robot pose and will not do anything
     * once vision measurements have been obtained
     * 
     * @param pose
     *            Pose to force
     */
    public void forceOdometry(Pose2d pose) {
        m_SLAMClient.forceOdometry(pose, new Rotation2d(m_gyroInputs.headingDeg), getModulePositions());
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
        return m_slamRobotPose.toPose2d();
    }

    public Pose3d getEstimatorPose3d() {
        return m_slamRobotPose;
    }

    public Pose3d getDeadReckoningPose3d() {
        return new Pose3d(m_deadReckoning.getPoseMeters());
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                m_rearLeftModule.getPosition(), m_rearRightModule.getPosition() };
    }

    public void setHeadingController(boolean enabled) {
        m_headingControllerEnabled = enabled;
    }

    public void setHeading(Rotation2d heading) {
        setHeading = heading;
    }

    public void initializeSLAM() {
        aprilTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        Pose3d[] landmarks = new Pose3d[aprilTags.getTags().size()];

        for (int i = 1; i <= aprilTags.getTags().size(); i++) {
            landmarks[i - 1] = aprilTags.getTagPose(i).get();
        }

        m_SLAMClient.seedSLAMLandmarks(landmarks);

        m_SLAMClient.setVisionConstants(
                SLAMConstantsConfig.camTransforms,
                SLAMConstantsConfig.POSE_VARIANCE_SCALE,
                SLAMConstantsConfig.POSE_VARIANCE_STATIC);
    }
}