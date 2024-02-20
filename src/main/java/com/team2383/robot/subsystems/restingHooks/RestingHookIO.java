package com.team2383.robot.subsystems.restingHooks;

import org.littletonrobotics.junction.AutoLog;

public interface RestingHookIO {
    @AutoLog
    public static class RestingHookIOInputs {
        public double voltage = 0.0;
        public double current = 0.0;
    }

    public default void updateInputs(RestingHookIOInputs inputs) {
    }

    public default void setPower(double power) {
    }
}
