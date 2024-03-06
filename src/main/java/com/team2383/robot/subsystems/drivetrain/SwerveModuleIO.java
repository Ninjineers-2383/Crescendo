package com.team2383.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public boolean angleMotorConnected = true;
        public boolean driveMotorConnected = true;
        public boolean angleEncoderConnected = true;

        public double driveVelocityMPS = 0.0;
        public double azimuthVelocityRPM = 0.0;
        public double angleRad = 0.0;
        public double absoluteAngleRad = 0.0;
        public double drivePositionM = 0.0;
        public double appliedVoltsDrive = 0.0;
        public double appliedVoltsAngle = 0.0;
        public double currentDrive = 0.0;
        public double currentAngle = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(SwerveModuleIOInputs inputs) {
    }

    public default void resetToAbsolute() {
    }

    public default void setDesiredState(SwerveModuleState desiredState) {
    }

    public default void stop() {
    }

    public default void setVoltage(double voltage) {
    }
}
