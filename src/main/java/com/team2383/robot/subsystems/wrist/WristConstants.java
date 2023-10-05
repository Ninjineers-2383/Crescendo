package com.team2383.robot.subsystems.wrist;

import edu.wpi.first.math.util.Units;

public final class WristConstants {
    public static final int kWristMotorID = 13;
    public static final int kAbsEncoderID = 5;

    public static final double kS = 0.0;
    public static final double kV = 2.83;
    public static final double kA = 0.07;

    public static final double kP = 12;
    public static final double kI = 1.2;
    public static final double kD = 4;

    public static final double kMaxVoltage = 12.0;
    public static final double kMaxVelocity = 5;
    public static final double kMaxAcceleration = 7;
    public static final int kMaxCurrent = 40;

    public static final double kMaxPosition = Units.inchesToMeters(45);
    public static final double kMinPosition = 0.0;

    public static final double kPositionTolerance = 0.01;

    public static final double kEncoderMetersPerRev = 0.5;

    public static final double kEncoderOffset = 3.365 / (2 * Math.PI);
    public static final double kWristMotorGearRatio = 1 / 85.33;

}
