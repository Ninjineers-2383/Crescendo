package com.team2383.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

    @AutoLog
    public static class WristIOInputs {
        public double velocityRadPerSec = 0.0;
        public double wristAngle = 0.0;
        public double absoluteWristAngle = 0.0;
        public double absoluteWristAngleOffset = 0.0;
        public double appliedVolts = 0.0;
        public double current = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(WristIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void forceAngle(double angle) {
    }

}
