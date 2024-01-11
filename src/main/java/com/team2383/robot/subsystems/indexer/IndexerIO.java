package com.team2383.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double leftPower = 0.0;
        public double rightPower = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {
    }

    public default void setLeftPower(double power) {
    }

    public default void setRightPower(double power) {
    }
}
