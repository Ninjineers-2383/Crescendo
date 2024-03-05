package com.team2383.robot.subsystems.piece_detection;

import org.littletonrobotics.junction.AutoLog;

public interface PieceDetectionIO {
    @AutoLog
    public static class PieceDetectionIOInputs {
        public boolean connected = false;

        public double frontYaw = 0.0;
        public boolean frontSeesTarget = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(PieceDetectionIOInputs inputs) {
    }
}
