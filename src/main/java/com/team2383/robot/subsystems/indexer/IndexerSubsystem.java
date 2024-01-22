package com.team2383.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private final IndexerIO indexer;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public IndexerSubsystem(IndexerIO io) {
        indexer = io;
    }

    @Override
    public void periodic() {
        indexer.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
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

}
