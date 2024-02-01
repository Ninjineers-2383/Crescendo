package com.team2383.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs {
        public double velocityRadPerS = 0.0;
        public double pivotAngle = 0.0;
        public double appliedVolts = 0.0;
        public double current = 0.0;
        public double desiredAngle = 0.0;
        public double desiredVelocity = 0.0;
        public double currentVelocity = 0.0;
        public double desiredAcceleration = 0.0;
        public double currentAcceleration = 0.0;
    }

    public default void updateInputs(PivotIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setAngle(double angle) {
    }

    public default void forceAngle(double angle) {
    }

    public default void setPIDController(PIDController controller) {
    }

    public default void setFeedforward(ArmFeedforward feedforward) {
    }
}
