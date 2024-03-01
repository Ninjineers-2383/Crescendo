package com.team2383.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class PivotConstants {
    // TODO: put correct numbers
    public static int kLeftMotorID = 3;
    public static int kRightMotorID = 4;
    public static int kEncoderID = 5;
    public static double kEncoderOffset = -0.961670;

    // feedforward constants
    public static final double kG = 0.4;
    public static final double kS = 2;
    public static final double kV = 70;
    public static final double kA = 25;

    public static final ArmFeedforward kFeedforwardController = new ArmFeedforward(kS, kG, kV, kA);

    // PID constants
    public static final double kP = 129;
    public static final double kI = 11;
    public static final double kD = 0;

    public static final PIDController kPIDController = new PIDController(kP, kI, kD);

    // Trapezoid Profile Constants
    public static double kMaxVelo = 60;
    public static double kMaxAccel = 20;

    public static double kPivotMotorGearRatio = 225;

}
