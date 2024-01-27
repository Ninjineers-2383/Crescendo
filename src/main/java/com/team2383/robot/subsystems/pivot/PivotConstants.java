package com.team2383.robot.subsystems.pivot;

public class PivotConstants {
    // TODO: put correct numbers

    public static int kLeftMotorID = 3;
    public static int kRightMotorID = 4;
    public static int kEncoderID = 5;

    // feedforward constants
    public static final double kG = 0.25;
    public static final double kS = 0;
    public static final double kV = 2.33;
    public static final double kA = 0.0;

    // PID constants
    public static final double kP = 4;
    public static final double kI = 0;
    public static final double kD = 0.25;

    // Trapezoid Profile Constants
    public static double kMaxVelo = 30;
    public static double kMaxAccel = 12;

    public static double kPivotMotorGearRatio = 25.0;

}
