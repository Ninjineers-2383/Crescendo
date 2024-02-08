package com.team2383.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class PivotConstants {
    // TODO: put correct numbers
    public static int kLeftMotorID = 3;
    public static int kRightMotorID = 4;
    public static int kEncoderID = 5;
    public static double kEncoderOffset = 0.589844 - (1 / 6.0);

    // feedforward constants
    public static final double kG = 0.4;
    public static final double kS = 0.7;
    public static final double kV = 6.9;
    public static final double kA = 1.5;

    public static final ArmFeedforward kFeedforwardController = new ArmFeedforward(kS, kG, kV, kA);

    // PID constants
    public static final double kP = 18;
    public static final double kI = 7;
    public static final double kD = 0;

    public static final PIDController kPIDController = new PIDController(kP, kI, kD);

    // Trapezoid Profile Constants
    public static double kMaxVelo = 30;
    public static double kMaxAccel = 12;

    public static double kPivotMotorGearRatio = 75.0;

}
