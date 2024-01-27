package com.team2383.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double topVoltage = 0.0;
        public double bottomVoltage = 0.0;
        public double sideVoltage = 0.0;

        public double topCurrent = 0.0;
        public double bottomCurrent = 0.0;
        public double sideCurrent = 0.0;

        public double topPosition = 0.0;
        public double bottomPosition = 0.0;
        public double sidePosition = 0.0;

        public double topVelocity = 0.0;
        public double bottomVelocity = 0.0;
        public double sideVelocity = 0.0;

        public double topBottomSetpointRPM = 0.0;
        public double sideSetpointRPM = 0.0;

    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setTopBottomRPM(double RPM, double differential) {
    }

    public default void setSideRPM(double RPM) {
    }

    public default void setTopBottomVoltage(double voltage) {
    }

    public default void setSideVoltage(double voltage) {
    }
}
