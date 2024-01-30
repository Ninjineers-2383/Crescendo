package com.team2383.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
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
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import java.nio.file.Path;

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
    private final ProfiledPIDController m_headingController = DriveConstants.HEADING_CONTROLLER;
    private Rotation2d desiredHeading = new Rotation2d();

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Distance> m_distanceDrive = mutable(Meters.of(0));

    private final MutableMeasure<Angle> m_distanceAngle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocityDrive = mutable(MetersPerSecond.of(0));

    private final MutableMeasure<Velocity<Angle>> m_velocityAngle = mutable(RotationsPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine;

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

        Translation2d[] moduleLocations = new Translation2d[] { DriveConstants.frontLeftConstants.translation,
                DriveConstants.frontRightConstants.translation, DriveConstants.rearLeftConstants.translation,
                DriveConstants.rearRightConstants.translation };

        try {
            aprilTags = new AprilTagFieldLayout(Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "out.json"));
        } catch (Exception e) {
            aprilTags = new AprilTagFieldLayout(null, 0, 0);
        }

        Pose3d[] landmarks = new Pose3d[aprilTags.getTags().size()];
        for (int i = 1; i <= aprilTags.getTags().size(); i++) {
            landmarks[i - 1] = aprilTags.getTagPose(i).get();
        }

        m_SLAMClient = new SLAMClient(new SLAMIOServer(moduleLocations, landmarks));

        initializeSLAM();

        SmartDashboard.putData("Field", m_field);
        m_COR = m_field.getObject("COR");

        m_gyro.setHeading(new Rotation2d());

        m_headingController.enableContinuousInput(-Math.PI, Math.PI);
        m_headingController.reset(0);

        forceHeading(new Rotation2d());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::forceOdometry,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                DriveConstants.CONFIG,
                () -> true,
                this);

        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motors.
                        (Measure<Voltage> volts) -> {
                            m_frontLeftModule.setVoltage(volts.in(Volts));
                            m_frontRightModule.setVoltage(volts.in(Volts));
                            m_rearLeftModule.setVoltage(volts.in(Volts));
                            m_rearRightModule.setVoltage(volts.in(Volts));
                        },
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being
                        // characterized.
                        log -> {
                            // Record a frame for the left motors. Since these share an encoder, we consider
                            // the entire group to be one motor.
                            m_frontLeftModule.periodic();
                            m_frontRightModule.periodic();
                            m_rearLeftModule.periodic();
                            m_rearRightModule.periodic();

                            log.motor("Front Left Drive")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    m_frontLeftModule.m_inputs.appliedVoltsDrive, Volts))
                                    .linearPosition(
                                            m_distanceDrive.mut_replace(m_frontLeftModule.m_inputs.drivePositionM,
                                                    Meters))
                                    .linearVelocity(
                                            m_velocityDrive.mut_replace(m_frontLeftModule.m_inputs.driveVelocityMPS,
                                                    MetersPerSecond));

                            log.motor("Front Left Azimuth")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    m_frontLeftModule.m_inputs.appliedVoltsAngle, Volts))
                                    .angularPosition(m_distanceAngle.mut_replace(m_frontLeftModule.m_inputs.angleRad,
                                            Radians))
                                    .angularVelocity(
                                            m_velocityAngle.mut_replace(
                                                    m_frontLeftModule.m_inputs.azimuthVelocityRPM / 60.0,
                                                    RotationsPerSecond));

                            log.motor("Front Right Drive")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    m_frontRightModule.m_inputs.appliedVoltsDrive, Volts))
                                    .linearPosition(
                                            m_distanceDrive.mut_replace(m_frontRightModule.m_inputs.drivePositionM,
                                                    Meters))
                                    .linearVelocity(
                                            m_velocityDrive.mut_replace(m_frontRightModule.m_inputs.driveVelocityMPS,
                                                    MetersPerSecond));

                            log.motor("Front Right Azimuth")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    m_frontRightModule.m_inputs.appliedVoltsAngle, Volts))
                                    .angularPosition(m_distanceAngle.mut_replace(m_frontRightModule.m_inputs.angleRad,
                                            Radians))
                                    .angularVelocity(
                                            m_velocityAngle.mut_replace(
                                                    m_frontRightModule.m_inputs.azimuthVelocityRPM / 60.0,
                                                    RotationsPerSecond));

                            log.motor("Rear Left Drive")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    m_rearLeftModule.m_inputs.appliedVoltsDrive, Volts))
                                    .linearPosition(
                                            m_distanceDrive.mut_replace(m_rearLeftModule.m_inputs.drivePositionM,
                                                    Meters))
                                    .linearVelocity(
                                            m_velocityDrive.mut_replace(m_rearLeftModule.m_inputs.driveVelocityMPS,
                                                    MetersPerSecond));

                            log.motor("Rear Left Azimuth")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    m_rearLeftModule.m_inputs.appliedVoltsAngle, Volts))
                                    .angularPosition(m_distanceAngle.mut_replace(m_rearLeftModule.m_inputs.angleRad,
                                            Radians))
                                    .angularVelocity(
                                            m_velocityAngle.mut_replace(
                                                    m_rearLeftModule.m_inputs.azimuthVelocityRPM / 60.0,
                                                    RotationsPerSecond));

                            log.motor("Rear Right Drive")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    m_rearRightModule.m_inputs.appliedVoltsDrive, Volts))
                                    .linearPosition(
                                            m_distanceDrive.mut_replace(m_rearRightModule.m_inputs.drivePositionM,
                                                    Meters))
                                    .linearVelocity(
                                            m_velocityDrive.mut_replace(m_rearRightModule.m_inputs.driveVelocityMPS,
                                                    MetersPerSecond));

                            log.motor("Rear Right Azimuth")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    m_rearRightModule.m_inputs.appliedVoltsAngle, Volts))
                                    .angularPosition(m_distanceAngle.mut_replace(m_rearRightModule.m_inputs.angleRad,
                                            Radians))
                                    .angularVelocity(
                                            m_velocityAngle.mut_replace(
                                                    m_rearRightModule.m_inputs.azimuthVelocityRPM / 60.0,
                                                    RotationsPerSecond));
                        },
                        // Tell SysId to make generated commands require this subsystem, suffix test
                        // state in
                        // WPILog with this subsystem's name
                        this));

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

        double headingEffort = m_headingController.calculate(getHeading().getRadians(), desiredHeading.getRadians());
        setChassisSpeedsSetpoint(new ChassisSpeeds(m_robotRelativeChassisSpeeds.vxMetersPerSecond,
                m_robotRelativeChassisSpeeds.vyMetersPerSecond,
                headingEffort));

        // m_robotRelativeChassisSpeeds = m_kinematics.toChassisSpeeds(m_lastStates);

        headingIntegral += headingEffort * 0.02;
        // headingIntegral %= (Math.PI);

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
            Translation2d centerOfRotation, boolean enableHeadingControl) {
        if (fieldRelative) {
            m_robotRelativeChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(drive.getX(), drive.getY(),
                    angle.getRadians(),
                    getHeading());
        } else {
            m_robotRelativeChassisSpeeds = new ChassisSpeeds(drive.getX(), drive.getY(), angle.getRadians());
        }

        desiredHeading = desiredHeading.plus(new Rotation2d(m_robotRelativeChassisSpeeds.omegaRadiansPerSecond * 0.02));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_robotRelativeChassisSpeeds;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(m_robotRelativeChassisSpeeds, getHeading());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        m_robotRelativeChassisSpeeds = speeds;

        desiredHeading = desiredHeading.plus(new Rotation2d(m_robotRelativeChassisSpeeds.omegaRadiansPerSecond * 0.02));
    }

    public void setChassisSpeedsSetpoint(ChassisSpeeds speeds) {
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
        if (m_gyroInputs.connected) {
            return Rotation2d.fromDegrees(m_gyroInputs.headingDeg).plus(new Rotation2d());
            // return Rotation2d.fromRadians(m_slamRobotPose.getRotation().getZ());
        } else {
            return Rotation2d.fromRadians(headingIntegral);
        }
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
        desiredHeading = currentHeading;
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

    public void setHeading(Rotation2d heading) {
        desiredHeading = heading;
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

    public Command getQuasiStatic(Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command getDynamic(Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}