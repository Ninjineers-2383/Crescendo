package com.team2383.robot.subsystems.pivot;

public class PivotConstants {
    // CAN ID's
    public static int kLeftMotorID = 3;
    public static int kRightMotorID = 4;
    public static int kEncoderID = 5;

    // Encoder Offset
    public static double kEncoderOffset = -0.455811;

    // Feedback and Feedforward Gains
    public static final ArmGains kGains = new ArmGains(40, 18.0, 0, 0.03, 5.1, 0, 0.45);

    // Trapezoid Profile Constants (In rotations / s and rotations / s^2)
    public static double kMaxVelo = 4;
    public static double kMaxAccel = 2;

    // Gear Ratio
    public static double kPivotMotorGearRatio = 48.0;

    // Pivot Bounds
    public static double kMaxAngleDegrees = 180;
    public static double kMinAngleDegrees = -55;

    // Current Limits
    public static double kCurrentLimit = 40;

    public record ArmGains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    }
}
