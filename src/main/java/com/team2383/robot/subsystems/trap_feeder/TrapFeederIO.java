package com.team2383.robot.subsystems.trap_feeder;

import org.littletonrobotics.junction.AutoLog;

public interface TrapFeederIO {
    @AutoLog
    public static class TrapFeederIOInputs {
        public double power = 0.0;
        public double current = 0.0;
    }

    public default void updateInputs(TrapFeederIOInputs inputs) {
    }

    public default void setPower(double power) {
    }
}
