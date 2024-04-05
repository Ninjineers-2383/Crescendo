package com.team2383.robot.subsystems.drivetrain.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    public static class VisionIOInputs implements LoggableInputs {
        public double[] timestamps = new double[] {};
        public double[][] frames = new double[][] {};
        public long fps = 0;

        @Override
        public void toLog(LogTable table) {
            int count = timestamps.length;
            table.put("length", count);
            for (int i = 0; i < count; i++) {
                table.put("timestamp/" + i, timestamps[i]);
                table.put("frame/" + i, frames[i]);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            int count = table.get("length", 0);
            timestamps = new double[count];
            frames = new double[0][count];
            for (int i = 0; i < count; i++) {
                frames[i] = table.get("frame/" + i, new double[0]);
                timestamps[i] = table.get("timestamp/" + i, 0.0);
            }
        }
    }

    public default void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
    }

    public default void setTagPoses(Pose3d[] tagPoses) {
    }
}
