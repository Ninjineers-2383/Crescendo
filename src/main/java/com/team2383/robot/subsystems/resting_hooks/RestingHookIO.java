package com.team2383.robot.subsystems.resting_hooks;

import org.littletonrobotics.junction.AutoLog;

public interface RestingHookIO {
    @AutoLog
    public static class RestingHookIOInputs {
        public boolean leftHookConnected = true;
        public boolean rightHookConnected = true;

        public double voltage = 0.0;
        public double current = 0.0;
    }

    public default void updateInputs(RestingHookIOInputs inputs) {
    }

    public default void setPower(double power) {
    }

    public default void setPowerSingle(double power) {
    }
}
