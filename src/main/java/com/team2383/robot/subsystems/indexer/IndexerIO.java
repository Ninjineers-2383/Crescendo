package com.team2383.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public boolean motorConnected = true;

        public double power = 0.0;
        public double supplyVoltage = 0.0;
        public double supplyCurrent = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {
    }

    public default void setPower(double power) {
    }
}
