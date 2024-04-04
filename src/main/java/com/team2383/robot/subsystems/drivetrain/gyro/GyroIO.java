package com.team2383.robot.subsystems.drivetrain.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double headingDeg = 0.0;
        public double rollDeg = 0.0;
        public double headingRateDPS = 0.0;
        public double headingAdjustment = 0.0;

    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(GyroIOInputs inputs) {
    }

    public default void resetHeading() {
    }

    public default void setHeading(Rotation2d heading) {
    }
}
