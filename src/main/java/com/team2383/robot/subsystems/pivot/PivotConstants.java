package com.team2383.robot.subsystems.pivot;

public class PivotConstants {
    // CAN ID's
    public static int kLeftMotorID = 3;
    public static int kRightMotorID = 4;
    public static int kEncoderID = 5;

    // Encoder Offset
    // 0.452881
    // 0.455811
    public static double kEncoderOffset = -0.284912;

    // Feedback and Feedforward Gains
    public static final ArmGains kGains = new ArmGains(20.0, 13.0, 2.0, 0.25, 6.55, 0.0, 0.3, 0.0, 4.0);

    // Trapezoid Profile Constants (In rotations / s and rotations / s^2)
    public static double kMaxVelo = 4.0;
    public static double kMaxAccel = 1.25;

    // Gear Ratio
    public static double kPivotMotorGearRatio = 60.0;

    // Pivot Bounds
    public static double kMaxAngleDegrees = 190;
    public static double kMinAngleDegrees = -30;

    // Current Limits
    public static double kCurrentLimit = 40;

    public record ArmGains(double kP, double kI, double kD, double kS, double kV, double kA, double kG,
            double kSpring, double kBacklash) {
    }
}
