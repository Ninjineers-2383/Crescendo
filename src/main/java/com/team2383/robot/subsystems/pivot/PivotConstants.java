package com.team2383.robot.subsystems.pivot;

public class PivotConstants {
    // TODO: put correct numbers

    public static int kLeftMotorID = 3;
    public static int kRightMotorID = 4;

    // feedforward constants
    public static double kS = 0.1;
    public static double kG = 0.1;
    public static double kV = 0.1;
    public static double kA = 0.1;

    // PID constants
    public static double kP = 5.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // Trapezoid Profile Constants
    public static double kMaxVelo = 1.0;
    public static double kMaxAccel = 1.0;

    public static double kPivotMotorGearRatio = 1.0 / 100.0;

}
