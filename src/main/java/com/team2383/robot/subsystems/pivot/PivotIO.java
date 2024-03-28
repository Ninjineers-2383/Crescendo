package com.team2383.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs {
        public boolean leftMotorConnected = true;
        public boolean rightMotorConnected = true;
        public boolean encoderConnected = true;

        public double rotorPositionRot = 0.0;
        public double absoluteEncoderPositionRot = 0.0;
        public double desiredPositionRot = 0.0;

        public double velocityRotPerSec = 0.0;
        public double desiredVelocityRotPerSec = 0.0;

        public double[] appliedVolts = new double[] {};
        public double[] supplyCurrentAmps = new double[] {};
        public double[] torqueCurrentAmps = new double[] {};
        public double[] tempCelcius = new double[] {};
    }

    public default void updateInputs(PivotIOInputs inputs) {
    }

    public default void setAngleRot(double angle, double velocity, PivotSubsystem.LashState lashState) {
    }

    public default void forceAngle(double angle) {
    }

    public default void setPIDController(double kP, double kI, double kD) {
    }

    public default void setFeedforward(double kS, double kV, double kA, double kG, double kSpring) {
    }

    public default void disable() {
    }

    public default void setVoltage(double voltage) {
    }
}
