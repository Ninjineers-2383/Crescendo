package com.team2383.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs {
        public boolean leftMotorConnected = true;
        public boolean rightMotorConnected = true;
        public boolean encoderConnected = true;

        public double velocityRadPerS = 0.0;
        public double pivotAngle = 0.0;
        public double appliedVolts = 0.0;
        public double current = 0.0;
        public double desiredAngle = 0.0;
        public double desiredVelocity = 0.0;
        public double currentVelocity = 0.0;
        public double desiredAcceleration = 0.0;
        public double currentAcceleration = 0.0;
        public double currentDesiredAngle = 0.0;
    }

    public default void updateInputs(PivotIOInputs inputs) {
    }

    public default void setAngleRadians(double angle, double velocity) {
    }

    public default void forceAngle(double angle) {
    }

    public default void setPIDController(double kP, double kI, double kD) {
    }

    public default void setFeedforward(double kS, double kV, double kA, double kG) {
    }

    public default void disable() {
    }

    public default void setVoltage(double voltage) {
    }
}
