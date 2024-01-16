package com.team2383.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double leftVoltage = 0.0;
        public double rightVoltage = 0.0;

        public double leftCurrent = 0.0;
        public double rightCurrent = 0.0;

        public double leftPosition = 0.0;
        public double rightPosition = 0.0;

        public double leftVelocity = 0.0;
        public double rightVelocity = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setRPM(double power) {
    }

    public default void setVoltage(double voltage) {
    }
}
