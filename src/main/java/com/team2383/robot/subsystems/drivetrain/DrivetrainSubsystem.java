package com.team2383.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.Optional;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team2383.lib.swerve.ModuleLimits;
import com.team2383.lib.swerve.SwerveSetpoint;
import com.team2383.lib.swerve.SwerveSetpointGenerator;
import com.team2383.lib.util.mechanical_advantage.Alert;
import com.team2383.lib.util.mechanical_advantage.Alert.AlertType;
import com.team2383.robot.Constants;
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

    private Rotation2d headingOffset = new Rotation2d();

    // Gyro Initialization
    private final GyroIO m_gyro;
    private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();

    // SLAM Initialization
    private SLAMClient m_SLAMClient;
    private AprilTagFieldLayout aprilTags;
    private Pose3d m_slamRobotPose = new Pose3d();

    // Field Sim Initialization
    private final Field2d m_field = new Field2d();

    // Robot Control Input Robot Relative
    private ChassisSpeeds m_robotRelativeChassisSpeeds = new ChassisSpeeds();

    // Heading Controller Initialization
    private ProfiledPIDController m_headingController = DriveConstants.HEADING_CONTROLLER;
    private Rotation2d desiredHeading = new Rotation2d();
    private boolean headingControllerEnabled = true;
    private boolean useManualHeadingTarget = false;

    private Rotation2d overrideHeading = new Rotation2d();
    private boolean overrideHeadingEnabled = false;

    private final SwerveSetpointGenerator setpointGenerator;
    private final ModuleLimits currentModuleLimits = DriveConstants.kModuleLimits;
    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
            });

    private Optional<Alliance> previousAlliance = Optional.empty();

    private Alert gyroConnected;

    private Alert frontLeftDriveConnected;
    private Alert frontLeftEncoderConnected;
    private Alert frontLeftAzimuthConnected;

    private Alert frontRightDriveConnected;
    private Alert frontRightEncoderConnected;
    private Alert frontRightAzimuthConnected;

    private Alert rearLeftDriveConnected;
    private Alert rearLeftEncoderConnected;
    private Alert rearLeftAzimuthConnected;

    private Alert rearRightDriveConnected;
    private Alert rearRightEncoderConnected;
    private Alert rearRightAzimuthConnected;

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

        try {
            aprilTags = new AprilTagFieldLayout(
                    Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "out.json"));
        } catch (Exception e) {
            aprilTags = new AprilTagFieldLayout(null, 0, 0);
        }

        SmartDashboard.putData("Field", m_field);

        m_gyro.setHeading(new Rotation2d());

        m_headingController.enableContinuousInput(-Math.PI, Math.PI);
        m_headingController.reset(0);

        reinitializeSLAM();

        forceHeading(new Rotation2d());

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::forceOdometry,
                this::getRobotRelativeSpeeds,
                (ChassisSpeeds speeds) -> {
                    drive(speeds, false, false);
                },
                DriveConstants.CONFIG,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        setpointGenerator = new SwerveSetpointGenerator(m_kinematics, DriveConstants.frontLeftConstants.translation,
                DriveConstants.frontRightConstants.translation,
                DriveConstants.rearLeftConstants.translation,
                DriveConstants.rearRightConstants.translation);

        initializeWarnings();

        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> m_frontLeftModule.getState().angle.getRadians(),
                        null);
                builder.addDoubleProperty("Front Left Velocity",
                        () -> m_frontLeftModule.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> m_frontRightModule.getState().angle.getRadians(),
                        null);
                builder.addDoubleProperty("Front Right Velocity",
                        () -> m_frontRightModule.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> m_rearLeftModule.getState().angle.getRadians(),
                        null);
                builder.addDoubleProperty("Back Left Velocity", () -> m_rearLeftModule.getState().speedMetersPerSecond,
                        null);

                builder.addDoubleProperty("Back Right Angle", () -> m_rearRightModule.getState().angle.getRadians(),
                        null);
                builder.addDoubleProperty("Back Right Velocity",
                        () -> m_rearRightModule.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
            }
        });
    }

    @Override
    public void periodic() {
        refreshWarnings();
        if (DriverStation.isDisabled()) {
            for (CoaxialSwerveModule module : m_modules) {
                module.stop();
            }
        }

        m_gyro.updateInputs(m_gyroInputs);
        Logger.processInputs("Gyro", m_gyroInputs);

        for (CoaxialSwerveModule module : m_modules) {
            module.periodic();
        }

        for (int i = 0; i < m_modules.length; i++) {
            m_lastStates[i] = m_modules[i].getState();
        }

        if (DriverStation.getAlliance().isPresent() && previousAlliance.isEmpty()) {
            reinitializeSLAM();
        } else if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() != previousAlliance.get()) {
            reinitializeSLAM();
        }

        previousAlliance = DriverStation.getAlliance();

        // Heading PID Controller
        double headingEffort;
        if (headingControllerEnabled) {
            headingEffort = m_headingController.calculate(getHeading().getRadians(),
                    desiredHeading.getRadians());
        } else {
            headingEffort = m_robotRelativeChassisSpeeds.omegaRadiansPerSecond;
        }

        setChassisSpeedsSetpoint(new ChassisSpeeds(m_robotRelativeChassisSpeeds.vxMetersPerSecond,
                m_robotRelativeChassisSpeeds.vyMetersPerSecond,
                headingEffort));

        headingIntegral += headingEffort * 0.02;

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

        m_slamRobotPose = update.pose();

        m_field.setRobotPose(update.pose().toPose2d());

        SmartDashboard.putNumber("Roll", getRoll());

        Logger.recordOutput("Swerve/Real Module States", m_lastStates);

        Logger.recordOutput("Swerve/Chassis Heading", getHeading().getRadians());
        Logger.recordOutput("Swerve/Desired Heading", desiredHeading.getRadians());
        Logger.recordOutput("Swerve/Heading Effort", headingEffort);
        Logger.recordOutput("Swerve/Chassis Heading Velocity", m_robotRelativeChassisSpeeds.omegaRadiansPerSecond);

        Logger.recordOutput("Robot Pose", m_slamRobotPose.toPose2d());

        Logger.recordOutput("SLAM/Robot Pose", m_slamRobotPose);
        // Logger.recordOutput("SLAM/landmarks", update.landmarks());
        Logger.recordOutput("SLAM/seenLandmarks", update.seenLandmarks());
        Logger.recordOutput("SLAM/newValue", update.newValue());

        Logger.recordOutput("Swerve/Dead Reckoning", m_deadReckoning.getPoseMeters());
    }

    private void reinitializeSLAM() {
        Translation2d[] moduleLocations = new Translation2d[] { DriveConstants.frontLeftConstants.translation,
                DriveConstants.frontRightConstants.translation, DriveConstants.rearLeftConstants.translation,
                DriveConstants.rearRightConstants.translation };

        aprilTags.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

        Pose3d[] landmarks = new Pose3d[aprilTags.getTags().size()];
        for (int i = 1; i <= aprilTags.getTags().size(); i++) {
            landmarks[i - 1] = aprilTags.getTagPose(i).get();
        }

        Logger.recordOutput("SLAM/landmarks", landmarks.clone());

        m_SLAMClient = new SLAMClient(new SLAMIOServer(moduleLocations, landmarks));

        m_SLAMClient.setVisionConstants(
                SLAMConstantsConfig.camTransforms,
                SLAMConstantsConfig.POSE_VARIANCE_SCALE,
                SLAMConstantsConfig.POSE_VARIANCE_STATIC);

        setSLAMHeading();
    }

    public void resetHeading() {
        headingOffset = Rotation2d.fromDegrees(m_gyroInputs.headingDeg).unaryMinus();
        m_headingController.reset(0);
        desiredHeading = new Rotation2d();
        setSLAMHeading();
    }

    private void setSLAMHeading() {
        Rotation2d SLAMHeading = desiredHeading;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            SLAMHeading = SLAMHeading.plus(new Rotation2d(Math.PI));
        }
        m_SLAMClient.forceHeading(SLAMHeading,
                Rotation2d.fromDegrees(m_gyroInputs.headingDeg),
                getModulePositions());
        System.out.println("SLAM Heading reset");

    }

    /**
     * Drive the robot using field or robot relative velocity
     * 
     * @param drive
     *            The speed for driving
     * @param fieldRelative
     *            Whether the speeds are relative to the field or the
     *            robot
     */
    public void drive(ChassisSpeeds drive, boolean fieldRelative, boolean useHeadingController) {
        if (fieldRelative) {
            m_robotRelativeChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(drive, getHeading());
        } else {
            m_robotRelativeChassisSpeeds = drive;
        }
        if (!headingControllerEnabled && useHeadingController && !useManualHeadingTarget) {
            desiredHeading = getHeading()
                    .plus(new Rotation2d(m_robotRelativeChassisSpeeds.omegaRadiansPerSecond * 0.02));
            m_headingController.reset(getHeading().getRadians());
        }
        headingControllerEnabled = useHeadingController;

        desiredHeading = desiredHeading.plus(new Rotation2d(m_robotRelativeChassisSpeeds.omegaRadiansPerSecond * 0.02));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_robotRelativeChassisSpeeds;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(m_robotRelativeChassisSpeeds, getHeading());
    }

    public void setChassisSpeedsSetpoint(ChassisSpeeds speeds) {
        currentSetpoint = setpointGenerator.generateSetpoint(currentModuleLimits, currentSetpoint, speeds,
                Constants.loopPeriodSecs);

        SwerveModuleState[] unoptimizedStates = m_kinematics.toSwerveModuleStates(
                speeds);

        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < m_modules.length; i++) {
            // Optimize setpoints
            optimizedSetpointStates[i] = SwerveModuleState.optimize(currentSetpoint.moduleStates()[i],
                    m_modules[i].getPosition().angle);
        }

        setModuleStates(optimizedSetpointStates);
        Logger.recordOutput("Swerve/Desired Module States", optimizedSetpointStates);

        Logger.recordOutput("Swerve/Unoptimized Module States", unoptimizedStates);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeed);

        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setDesiredState(states[i]);
        }
    }

    public void setModuleVoltages(Measure<Voltage> volts) {
        for (CoaxialSwerveModule module : m_modules) {
            module.setVoltage(volts.in(Volts));
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
        if (m_gyroInputs.connected) {
            return Rotation2d.fromDegrees(m_gyroInputs.headingDeg).plus(headingOffset);
        } else {
            return Rotation2d.fromRadians(headingIntegral);
        }
    }

    public CoaxialSwerveModule[] getModules() {
        return m_modules;
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
        headingOffset = Rotation2d.fromDegrees(m_gyroInputs.headingDeg).unaryMinus().plus(currentHeading);
        m_headingController.reset(currentHeading.getRadians());
        desiredHeading = currentHeading;
        setSLAMHeading();
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

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                m_rearLeftModule.getPosition(), m_rearRightModule.getPosition() };
    }

    public void setHeading(Rotation2d heading) {
        useManualHeadingTarget = true;
        headingControllerEnabled = true;
        desiredHeading = heading;
    }

    public void endManualHeadingControl() {
        useManualHeadingTarget = false;
    }

    public boolean shouldFlipPath() {
        if (!DriverStation.getAlliance().isPresent()) {
            return false;
        }

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return false;
        } else {
            return true;
        }
    }

    public boolean headingIsFinished() {
        return Math.abs(desiredHeading.minus(getHeading()).getRadians()) < 0.01;
    }

    public void setHeadingPID(ProfiledPIDController controller) {
        m_headingController = controller;
    }

    public void setRotationTargetOverride(Rotation2d desiredHeading) {
        overrideHeadingEnabled = true;
        overrideHeading = desiredHeading;
    }

    public void disableRotationTargetOverride() {
        overrideHeadingEnabled = false;
    }

    public Optional<Rotation2d> getRotationTargetOverride() {
        if (overrideHeadingEnabled) {
            return Optional.of(overrideHeading);
        } else {
            return Optional.empty();
        }
    }

    private void initializeWarnings() {
        gyroConnected = new Alert("Gyro Disconnected! CAN ID: " + 0, AlertType.ERROR);

        frontLeftDriveConnected = new Alert(
                "Front Left Drive Disconnected! CAN ID: " + DriveConstants.frontLeftConstants.kDriveMotorID,
                AlertType.ERROR);

        frontLeftEncoderConnected = new Alert(
                "Front Left Encoder Disconnected! CAN ID: " + 21,
                AlertType.ERROR);

        frontLeftAzimuthConnected = new Alert(
                "Front Left Azimuth Disconnected! CAN ID: " + DriveConstants.frontLeftConstants.kAngleMotorID,
                AlertType.ERROR);

        frontRightDriveConnected = new Alert(
                "Front Right Drive Disconnected! CAN ID: " + DriveConstants.frontRightConstants.kDriveMotorID,
                AlertType.ERROR);

        frontRightEncoderConnected = new Alert(
                "Front Right Encoder Disconnected! CAN ID: " + 24,
                AlertType.ERROR);

        frontRightAzimuthConnected = new Alert(
                "Front Right Azimuth Disconnected! CAN ID: " + DriveConstants.frontRightConstants.kAngleMotorID,
                AlertType.ERROR);

        rearLeftDriveConnected = new Alert(
                "Rear Left Drive Disconnected! CAN ID: " + DriveConstants.rearLeftConstants.kDriveMotorID,
                AlertType.ERROR);

        rearLeftEncoderConnected = new Alert(
                "Rear Left Encoder Disconnected! CAN ID: " + 27,
                AlertType.ERROR);

        rearLeftAzimuthConnected = new Alert(
                "Rear Left Azimuth Disconnected! CAN ID: " + DriveConstants.rearLeftConstants.kAngleMotorID,
                AlertType.ERROR);

        rearRightDriveConnected = new Alert(
                "Rear Right Drive Disconnected! CAN ID: " + DriveConstants.rearRightConstants.kDriveMotorID,
                AlertType.ERROR);

        rearRightEncoderConnected = new Alert(
                "Rear Right Encoder Disconnected! CAN ID: " + 30,
                AlertType.ERROR);

        rearRightAzimuthConnected = new Alert(
                "Rear Right Azimuth Disconnected! CAN ID: " + DriveConstants.rearRightConstants.kAngleMotorID,
                AlertType.ERROR);

    }

    public void refreshWarnings() {
        gyroConnected.set(!m_gyroInputs.connected);

        frontLeftDriveConnected.set(!m_frontLeftModule.m_inputs.driveMotorConnected);
        frontLeftEncoderConnected.set(!m_frontLeftModule.m_inputs.angleEncoderConnected);
        frontLeftAzimuthConnected.set(!m_frontLeftModule.m_inputs.angleMotorConnected);

        frontRightDriveConnected.set(!m_frontRightModule.m_inputs.driveMotorConnected);
        frontRightEncoderConnected.set(!m_frontRightModule.m_inputs.angleEncoderConnected);
        frontRightAzimuthConnected.set(!m_frontRightModule.m_inputs.angleMotorConnected);

        rearLeftDriveConnected.set(!m_rearLeftModule.m_inputs.driveMotorConnected);
        rearLeftEncoderConnected.set(!m_rearLeftModule.m_inputs.angleEncoderConnected);
        rearLeftAzimuthConnected.set(!m_rearLeftModule.m_inputs.angleMotorConnected);

        rearRightDriveConnected.set(!m_rearRightModule.m_inputs.driveMotorConnected);
        rearRightEncoderConnected.set(!m_rearRightModule.m_inputs.angleEncoderConnected);
        rearRightAzimuthConnected.set(!m_rearRightModule.m_inputs.angleMotorConnected);
    }
}