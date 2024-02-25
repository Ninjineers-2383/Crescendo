package com.team2383.robot.subsystems.side_shooter;

import org.littletonrobotics.junction.AutoLog;

public interface SideShooterIO {
    @AutoLog
    public static class SideShooterIOInputs {
        public double voltage = 0.0;

        public double current = 0.0;

        public double position = 0.0;

        public double velocity = 0.0;

        public double setpointRPM = 0.0;

    }

    public default void updateInputs(SideShooterIOInputs inputs) {
    }

    public default void setRPM(double RPM) {
    }

    public default void setVoltage(double voltage) {
    }
}
