package com.team2383.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    @AutoLog
    public static class ElevatorIOInputs {
        public double velocityMPS = 0.0;
        public double positionM = 0.0;
        public double appliedVolts = 0.0;
        public double current = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void forcePosition(double positionMeters) {
    }

}
