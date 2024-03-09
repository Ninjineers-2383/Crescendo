package com.team2383.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public boolean motorConnected = true;

        public double power = 0.0;
        public double current = 0.0;

        public boolean beamBreakTripped = false;
    }

    public default void updateInputs(FeederIOInputs inputs) {
    }

    public default void setPower(double power) {
    }
}
