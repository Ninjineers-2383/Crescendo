package com.team2383.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs {
        public double velocityRadPerS = 0.0;
        public double pivotAngle = 0.0;
        public double appliedVolts = 0.0;
        public double current = 0.0;
    }

    public default void updateInputs(PivotIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void forceAngle(double angle) {
    }

}
