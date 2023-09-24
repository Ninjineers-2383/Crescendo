package com.team2383.robot.subsystems.drivetrain;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import com.team2383.lib.swerve.AbsoluteCancoder;
import com.team2383.lib.swerve.AbsoluteMagEncoder;
import com.team2383.lib.swerve.IAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final double kMaxSpeed = 3.8; // meters per second

    public final static double kTrackWidthMeters = 0.6173724;
    public final static double kDriveMaxVoltage = 9.0;
    public final static double kMaxCurrent = 30.0;

    public final static double kAngleGearRatio = 12.8 / 1.0;
    public final static double kDriveGearRatio = 6.75 / 1.0;

    public final static double kDriveWheelDiameterMeters = Units.inchesToMeters(4);
    public final static double kDriveWheelCircumferenceMeters = Math.PI * kDriveWheelDiameterMeters;

    public final static double kMaxAngularVelocity = Math.PI * 20;
    public final static double kMaxAngularAcceleration = Math.PI * 2 * 100;

    public final static PIDController HEADING_CONTROLLER = new PIDController(1, 0, 0);

    public static final class ModuleConstants {
        public final double kS;
        public final double kV;
        public final double kA;

        public final double kP;
        public final double kI;
        public final double kD;

        public final int kAngleMotorID;
        public final int kDriveMotorID;

        public final int kEncoderID;

        public final String name;
        public final Translation2d translation;

        public final Rotation2d kAngleOffset;

        public final HardwareConfigs kHardwareConfigs;

        public ModuleConstants(double kS, double kV, double kA,
                double kP, double kI, double kD,
                int kAngleMotorID, int kDriveMotorID, int kEncoderID,
                String name, Translation2d translation,
                Rotation2d angleOffset) {

            this.kS = kS;
            this.kV = kV;
            this.kA = kA;

            this.kP = kP;
            this.kI = kI;
            this.kD = kD;

            this.kAngleMotorID = kAngleMotorID;
            this.kDriveMotorID = kDriveMotorID;

            this.kEncoderID = kEncoderID;

            this.name = name;
            this.translation = translation;

            this.kAngleOffset = angleOffset;

            this.kHardwareConfigs = new HardwareConfigs(kP, kI, kD, kS, kV);
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

            kAngleMotorConfigs.MotorOutput = new MotorOutputConfigs();
            kAngleMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
    }

    public final static ModuleConstants frontLeftConstants = new ModuleConstants(
            0.065635,
            0.15,
            0.015968,
            0.27135, 0, 0,
            20, 21, 1,
            "frontLeft",
            new Translation2d(
                    (Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 4,
                    DriveConstants.kTrackWidthMeters / 2),
            Rotation2d.fromDegrees(230.625));

    public final static ModuleConstants frontRightConstants = new ModuleConstants(
            0.065635,
            0.15,
            0.015968,
            0.27135, 0, 0,
            22, 23, 2,
            "frontRight",
            new Translation2d(
                    (Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 4,
                    -DriveConstants.kTrackWidthMeters / 2),
            Rotation2d.fromDegrees(207.86));

    public final static ModuleConstants rearConstants = new ModuleConstants(
            0.065635,
            0.15,
            0.015968,
            0.27135, 0, 0,
            24, 25, 3,
            "rear",
            new Translation2d(
                    -(Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 4,
                    0),
            Rotation2d.fromDegrees(283.359375));

    public final static IAbsoluteEncoder frontLeftEncoder = new AbsoluteMagEncoder(0);
    public final static IAbsoluteEncoder frontRightEncoder = new AbsoluteCancoder(2, "Drive",
            frontRightConstants.kHardwareConfigs.kAngleEncoderConfigs);
    public final static IAbsoluteEncoder rearEncoder = new AbsoluteCancoder(3, "Drive",
            rearConstants.kHardwareConfigs.kAngleEncoderConfigs);

}
