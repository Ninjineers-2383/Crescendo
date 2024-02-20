package com.team2383.robot.subsystems.trap_arm;

import com.team2383.lib.controller.TunableArmFeedforward;

import edu.wpi.first.math.controller.PIDController;

public class TrapArmConstants {
    public static final int kPivotID = 0; // TODO: Set these to the correct values

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kS = 0;
    public static final double kG = 0;

    public static final PIDController kFeedbackController = new PIDController(kP, kI, kD);
    public static final TunableArmFeedforward kFeedforwardController = new TunableArmFeedforward(kS, kG, kV, kA);

}
