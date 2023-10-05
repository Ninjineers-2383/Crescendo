package com.team2383.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public final class ElevatorConstants {
    public static final double kS = 0.3;
    public static final double kV = 1.9;
    public static final double kA = 0.07;
    public static final double kG = 0.541;

    public static final double kP = 18.2;
    public static final double kI = 0.2;
    public static final double kD = 0.5;

    public static final int kElevatorLeftID = 11;
    public static final int kElevatorRightID = 12;

    public static final double kMaxVoltage = 12.0;
    public static final double kMaxVelocity = 5;
    public static final double kMaxAcceleration = 5;

    public static final double kMaxPosition = Units.inchesToMeters(45);
    public static final double kMinPosition = 0.0;

    public static final double kPositionTolerance = 0.01;

    public static final double kEncoderMetersPerRev = 0.5;

    public static final double kElevatorGearRatio = 0;

}
