package com.team2383.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.team2383.lib.util.mechanical_advantage.Alert;
import com.team2383.lib.util.mechanical_advantage.Alert.AlertType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private final IndexerIO indexer;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private final Alert motorDisconnected = new Alert(
            "Indexer Motor Disconnected! CAN ID: " + IndexerConstants.kIndexerID, AlertType.WARNING);

    public IndexerSubsystem(IndexerIO io) {
        indexer = io;
    }

    @Override
    public void periodic() {
        indexer.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        motorDisconnected.set(!inputs.motorConnected);
    }

    public void feed() {
        indexer.setPower(1);
    }

    public void stop() {
        indexer.setPower(0);
    }

    public void setPower(double power) {
        indexer.setPower(power);
    }

    public boolean isBeamBreakTripped() {
        return inputs.beamBreakTripped;
    }

}
