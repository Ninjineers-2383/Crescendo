package com.team2383.robot.subsystems.pivot;

public class PivotConstants {
    // CAN ID's
    public static int kLeftMotorID = 3;
    public static int kRightMotorID = 4;
    public static int kEncoderID = 5;

    // Encoder Offset
    // 0.452881
    // 0.455811
    public static double kEncoderOffset = -0.452881;

    // Feedback and Feedforward Gains
    public static final ArmGains kGains = new ArmGains(15, 10, 1, 0.1, 7.0, 0.0, 0.3, 0.01);

    // Trapezoid Profile Constants (In rotations / s and rotations / s^2)
    public static double kMaxVelo = 4;
    public static double kMaxAccel = 2;

    // Gear Ratio
    public static double kPivotMotorGearRatio = 60.0;

    // Pivot Bounds
    public static double kMaxAngleDegrees = 180;
    public static double kMinAngleDegrees = -55;

    // Current Limits
    public static double kCurrentLimit = 40;

    public record ArmGains(double kP, double kI, double kD, double kS, double kV, double kA, double kG,
            double kSpring) {
    }
}
