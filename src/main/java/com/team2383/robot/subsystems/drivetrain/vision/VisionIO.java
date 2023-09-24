package com.team2383.robot.subsystems.drivetrain.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean[] connected = {};
        public double[] x = {};
        public double[] y = {};
        public double[] z = {};

        public double[] roll = {};
        public double[] pitch = {};
        public double[] yaw = {};

        public double[] ambiguity = {};

        public double[] timestampSeconds = {};

        public void clear(int len) {
            connected = new boolean[len];
            x = new double[len];
            y = new double[len];
            z = new double[len];

            roll = new double[len];
            pitch = new double[len];
            yaw = new double[len];

            ambiguity = new double[len];

            timestampSeconds = new double[len];
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {
    }
}
