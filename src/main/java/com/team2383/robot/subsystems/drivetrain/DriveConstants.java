package com.team2383.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team2383.lib.swerve.AbsoluteCancoder;
import com.team2383.lib.swerve.IAbsoluteEncoder;
import com.team2383.lib.swerve.ModuleLimits;
import com.team2383.robot.Constants;
import com.team2383.robot.Constants.RobotType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final double kMaxSpeed = 3.8; // meters per second

    public final static double kTrackWidthMeters = Constants.getRobot() == RobotType.ROBOT_COMP
            ? Units.inchesToMeters(21.75)
            : Units.inchesToMeters(19.75);
    public final static double kWheelBaseMeters = kTrackWidthMeters;
    public final static double kDriveMaxVoltage = 9.0;
    public final static double kMaxCurrent = 30.0;

    public final static double kAngleGearRatio = ((150.0 / 7.0) / 1.0);
    public final static double kDriveGearRatio = 6.12 / 1.0;

    public final static double kDriveWheelDiameterMeters = Units.inchesToMeters(4);
    public final static double kDriveWheelCircumferenceMeters = Math.PI * kDriveWheelDiameterMeters;

    public final static double kMaxAngularVelocity = Math.PI * 20;
    public final static double kMaxAngularAcceleration = Math.PI * 2 * 100;

    public final static HolonomicPathFollowerConfig CONFIG = new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0, 0),
            new PIDConstants(4, 0, 0),
            4,
            0.5,
            new ReplanningConfig());

    public final static PathConstraints AUTO_CONSTRAINTS = new PathConstraints(
            1, 1,
            Units.degreesToRadians(180), Units.degreesToRadians(180));

    public final static ProfiledPIDController HEADING_CONTROLLER = new ProfiledPIDController(10.0, 0, 0,
            new Constraints(4 * Math.PI, 4 * Math.PI));

    public final static Pose2d SPEAKER_POSE = new Pose2d(new Translation2d(1.84, 7.71), Rotation2d.fromDegrees(270));

    public static final ModuleLimits kModuleLimits = new ModuleLimits(
            3, 6, 2);

    public static final class ModuleConstants {
        public final double kS;
        public final double kV;
        public final double kA;

        public final double kP;
        public final double kI;
        public final double kD;

        public final int kAngleMotorID;
        public final int kDriveMotorID;

        public final IAbsoluteEncoder kEncoder;

        public final String name;
        public final Translation2d translation;

        public final Rotation2d kAngleOffset;

        public final HardwareConfigs kHardwareConfigs;

        public final boolean invertDrive;
        public final boolean invertAzimuth;

        public ModuleConstants(double kS, double kV, double kA,
                double kP, double kI, double kD,
                int kAngleMotorID, int kDriveMotorID, IAbsoluteEncoder kEncoder,
                String name, Translation2d translation,
                Rotation2d angleOffset, boolean invertDrive, boolean invertAzimuth) {

            this.kS = kS;
            this.kV = kV;
            this.kA = kA;

            this.kP = kP;
            this.kI = kI;
            this.kD = kD;

            this.kAngleMotorID = kAngleMotorID;
            this.kDriveMotorID = kDriveMotorID;

            this.kEncoder = kEncoder;

            this.name = name;
            this.translation = translation;

            this.kAngleOffset = angleOffset;

            this.kHardwareConfigs = new HardwareConfigs(kP, kI, kD, kS, kV);

            this.invertDrive = invertDrive;
            this.invertAzimuth = invertAzimuth;
        }
    }

    public static final class HardwareConfigs {
        public TalonFXConfiguration kDriveMotorConfigs;
        public TalonFXConfiguration kAngleMotorConfigs;
        public CANcoderConfiguration kAngleEncoderConfigs;

        public HardwareConfigs(double kP, double kI, double kD, double kS, double kV) {
            kDriveMotorConfigs = new TalonFXConfiguration();
            kAngleMotorConfigs = new TalonFXConfiguration();
            kAngleEncoderConfigs = new CANcoderConfiguration();

            kDriveMotorConfigs.CurrentLimits = new CurrentLimitsConfigs();
            kDriveMotorConfigs.CurrentLimits.SupplyCurrentLimit = 65;
            kDriveMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

            kDriveMotorConfigs.Slot0 = new Slot0Configs();
            kDriveMotorConfigs.Slot0.kP = kP;
            kDriveMotorConfigs.Slot0.kI = kI;
            kDriveMotorConfigs.Slot0.kD = kD;

            kDriveMotorConfigs.Slot0.kS = kS;
            kDriveMotorConfigs.Slot0.kV = kV;
            kDriveMotorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

            kAngleEncoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            kAngleEncoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

            kAngleMotorConfigs.CurrentLimits = new CurrentLimitsConfigs();
            kAngleMotorConfigs.CurrentLimits.SupplyCurrentLimit = 65;
            kAngleMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

            kAngleMotorConfigs.Slot0 = new Slot0Configs();
            kAngleMotorConfigs.Slot0.kP = 0.8;
            kAngleMotorConfigs.Slot0.kD = 0.0;

            kAngleMotorConfigs.MotorOutput = new MotorOutputConfigs();
            kAngleMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
    }

    private static final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    public final static ModuleConstants frontLeftConstants = new ModuleConstants(
            0.065635,
            0.15,
            0.015968,
            0.01, 0, 0,
            20, 22,
            new AbsoluteCancoder(21, "Drive", cancoderConfig),
            "frontLeft",
            new Translation2d(
                    DriveConstants.kWheelBaseMeters / 2,
                    DriveConstants.kTrackWidthMeters / 2),
            Rotation2d.fromRotations(-0.060059),
            true, false);

    public final static ModuleConstants frontRightConstants = new ModuleConstants(
            0.065635,
            0.15,
            0.015968,
            0.01, 0, 0,
            23, 25,
            new AbsoluteCancoder(24, "Drive", cancoderConfig),
            "frontRight",
            new Translation2d(
                    DriveConstants.kWheelBaseMeters / 2,
                    -DriveConstants.kTrackWidthMeters / 2),
            Rotation2d.fromRotations(0.326660),
            false, true);

    public final static ModuleConstants rearLeftConstants = new ModuleConstants(
            0.065635,
            0.15,
            0.015968,
            0.01, 0, 0,
            26, 28,
            new AbsoluteCancoder(27, "Drive", cancoderConfig),
            "rearLeft",
            new Translation2d(
                    -DriveConstants.kWheelBaseMeters / 2,
                    DriveConstants.kTrackWidthMeters / 2),
            Rotation2d.fromRotations(-0.167236),
            false, true);

    public final static ModuleConstants rearRightConstants = new ModuleConstants(
            0.065635,
            0.15,
            0.015968,
            0.01, 0, 0,
            29, 31,
            new AbsoluteCancoder(30, "Drive", cancoderConfig),
            "rearRight",
            new Translation2d(
                    -DriveConstants.kWheelBaseMeters / 2,
                    -DriveConstants.kTrackWidthMeters / 2),
            Rotation2d.fromRotations(0.194580),
            true, false);

}
