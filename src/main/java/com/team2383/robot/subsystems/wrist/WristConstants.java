package com.team2383.robot.subsystems.wrist;

import edu.wpi.first.math.util.Units;

public final class WristConstants {
    public static final int kWristMotorID = 13;
    public static final int kAbsEncoderID = 5;

    public static final double kG = 0.25;
    public static final double kS = 0;
    public static final double kV = 1.9;
    public static final double kA = 0.0;

    public static final double kP = 3;
    public static final double kI = 0;
    public static final double kD = 0.25;

    public static final double kMaxVoltage = 12.0;
    public static final double kMaxVelocity = 30;
    public static final double kMaxAcceleration = 12;
    public static final int kMaxCurrent = 40;

    public static final double kMaxPosition = Units.inchesToMeters(45);
    public static final double kMinPosition = 0.0;

    public static final double kPositionTolerance = 0.1;

    public static final double kEncoderMetersPerRev = 0.5;

    public static final double kEncoderOffset = 2.404;
    public static final double kWristMotorGearRatio = (4.0 / 1) * (3.0 / 1) * (4.0 / 1) * (32.0 / 18.0);

}
