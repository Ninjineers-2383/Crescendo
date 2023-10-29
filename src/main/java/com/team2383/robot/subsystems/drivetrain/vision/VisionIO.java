package com.team2383.robot.subsystems.drivetrain.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean[] connected = {};
        public double[] x = {};
        public double[] y = {};
        public double[] z = {};

        public double[] rw = {};
        public double[] rx = {};
        public double[] ry = {};
        public double[] rz = {};

        public long[] tagId = {};

        public double[] ambiguity = {};

        public double[] timestampSeconds = {};

        public void clear(int len) {
            connected = new boolean[len];
            x = new double[len];
            y = new double[len];
            z = new double[len];

            rw = new double[len];
            rx = new double[len];
            ry = new double[len];
            rz = new double[len];

            tagId = new long[len];

            ambiguity = new double[len];

            timestampSeconds = new double[len];
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
    }
}
