package com.team2383.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

import com.team2383.robot.subsystems.drivetrain.vision.VisionIOInputsAutoLogged;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team2383.robot.subsystems.drivetrain.vision.VisionConstants;
import com.team2383.robot.subsystems.drivetrain.vision.VisionIO;

public class DrivetrainSubsystem extends SubsystemBase {
    private final CoaxialSwerveModule m_frontLeftModule;
    private final CoaxialSwerveModule m_frontRightModule;
    private final CoaxialSwerveModule m_rearLeftModule;
    private final CoaxialSwerveModule m_rearRightModule;

    private final CoaxialSwerveModule[] m_modules;
    private final SwerveModuleState[] m_lastStates;

    private final VisionIO m_vision;
    private final VisionIOInputsAutoLogged m_visionInputs = new VisionIOInputsAutoLogged();

    private final GyroIO m_gyro;
    private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();

    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            DriveConstants.frontLeftConstants.translation,
            DriveConstants.frontRightConstants.translation,
            DriveConstants.rearLeftConstants.translation,
            DriveConstants.rearRightConstants.translation);

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final Field2d m_field = new Field2d();
    private final FieldObject2d m_COR;

    private int loop_cycle = 0;

    private double headingIntegral = 0;

    public DrivetrainSubsystem(GyroIO gyro, VisionIO vision, SwerveModuleIO frontLeftIO, SwerveModuleIO frontRightIO,
            SwerveModuleIO rearLeftIO, SwerveModuleIO rearRightIO) {
        m_gyro = gyro;
        m_vision = vision;

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

        // headingIntegral =
        // m_poseEstimator.getEstimatedPosition().getRotation().getRadians();

    }

    @Override
    public void periodic() {
        if (loop_cycle == 200) {
            for (CoaxialSwerveModule module : m_modules) {
                module.resetToAbsolute();
            }
        }

        loop_cycle++;

        m_gyro.updateInputs(m_gyroInputs);
        Logger.getInstance().processInputs("Gyro", m_gyroInputs);

        m_vision.updateInputs(m_visionInputs);
        Logger.getInstance().processInputs("Vision", m_visionInputs);

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

        for (int i = 0; i < m_visionInputs.connected.length; i++) {
            if (!m_visionInputs.connected[i])
                continue;

            Translation3d camTranslation = new Translation3d(
                    m_visionInputs.x[i],
                    m_visionInputs.y[i],
                    m_visionInputs.z[i]);

            Rotation3d camRotation = new Rotation3d(
                    m_visionInputs.roll[i],
                    m_visionInputs.pitch[i],
                    m_visionInputs.yaw[i]);

            Pose3d camPose = new Pose3d(camTranslation, camRotation);

            double distance = camTranslation.getNorm();

            double variance = Math.pow(distance, 1) * VisionConstants.POSE_VARIANCE_SCALE
                    + VisionConstants.POSE_VARIANCE_STATIC;

            m_poseEstimator.addVisionMeasurement(camPose.toPose2d(),
                    m_visionInputs.timestampSeconds[i],
                    VecBuilder.fill(variance, variance, variance));
        }

        Pose2d estimatedPose = m_poseEstimator.getEstimatedPosition();

        m_field.setRobotPose(estimatedPose);

        SmartDashboard.putNumber("Roll", getRoll());

        Logger.getInstance().recordOutput("Swerve/Real Module States", m_lastStates);

        Logger.getInstance().recordOutput("Swerve/Heading Integaral", headingIntegral);

        Logger.getInstance().recordOutput("Swerve/Chassis Heading", chassis.omegaRadiansPerSecond);

        Logger.getInstance().recordOutput("Robot Pose", estimatedPose);
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

        Logger.getInstance().recordOutput("Swerve/Desired Module States", swerveModuleStates);

        m_COR.setPose(getPose().plus(new Transform2d(centerOfRotation, new Rotation2d())));
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

    public Pose2d correctToAlliance(Pose2d pose) {
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            Translation2d transformedTranslation = new Translation2d(pose.getX(),
                    8.02 - pose.getY());
            Rotation2d transformedHeading = pose.getRotation().plus(Rotation2d.fromDegrees(180));

            pose = new Pose2d(transformedTranslation, transformedHeading);
        }

        return pose;
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
        m_gyro.setHeading(currentHeading);
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
        m_gyro.setHeading(Rotation2d.fromDegrees(getCompassHeading()));
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

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                m_rearLeftModule.getPosition(), m_rearRightModule.getPosition() };
    }

    public void forceOdometry(Pose2d pose) {
        correctToAlliance(pose);
        forceHeading(pose.getRotation());
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
        resetEncoders();
    }

    public void setPosition(Translation2d position) {
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(), new Pose2d(position, getHeading()));
        resetEncoders();
    }

    public double getCompassHeading() {
        // Compass is CW positive not CCW positive
        // double fieldCompassHeading = Preferences.getDouble("Compass", 0);
        // double currentCompassHeading = m_gyro.getComas().getValue();

        // return currentCompassHeading - fieldCompassHeading;
        return 0;
    }

    public void setCompassOffset() {
        // Compass is CW positive not CCW positive
        // double fieldCompassHeading = m_gyro.getCompassHeading();
        // Preferences.setDouble("Compass", fieldCompassHeading);

        // DataLogManager.log("INFO: Compass offset set to " + fieldCompassHeading +
        // "\n");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}