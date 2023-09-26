package com.team2383.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static final double kS = 0.0;
    public static final double kV = 2.83;
    public static final double kA = 0.07;

    public static final double kP = 12;
    public static final double kI = 1.2;
    public static final double kD = 4;

    public static final double kMaxVoltage = 12.0;
    public static final double kMaxVelocity = 5;
    public static final double kMaxAcceleration = 7;

    public static final double kMaxPosition = Units.inchesToMeters(45);
    public static final double kMinPosition = 0.0;

    public static final double kPositionTolerance = 0.01;

    public static final double kEncoderMetersPerRev = 0.5;
}
