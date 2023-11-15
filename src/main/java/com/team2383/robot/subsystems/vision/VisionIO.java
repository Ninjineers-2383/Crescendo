package com.team2383.robot.subsystems.vision;

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
            table.put("Timestamps", timestamps);
            table.put("FrameCount", frames.length);
            for (int i = 0; i < frames.length; i++) {
                table.put("Frame/" + Integer.toString(i), frames[i]);
            }
            table.put("Fps", fps);
        }

        @Override
        public void fromLog(LogTable table) {
            timestamps = table.get("Timestamps", timestamps);
            int frameCount = (int) table.get("FrameCount", 0l);
            frames = new double[frameCount][];
            for (int i = 0; i < frameCount; i++) {
                frames[i] = table.get("Frame/" + Integer.toString(i), new double[] {});
            }
            fps = table.get("Fps", fps);
        }
    }

    public default void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
    }
}
