package com.team2383.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double power = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {
    }

    public default void setPower(double power) {
    }
}
