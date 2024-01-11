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

    public void feedLeft() {
        indexer.setLeftPower(1);
        indexer.setRightPower(0);
    }

    public void feedRight() {
        indexer.setLeftPower(0);
        indexer.setRightPower(1);
    }

    public void shoot() {
        indexer.setLeftPower(1);
        indexer.setRightPower(1);
    }

    public void stop() {
        indexer.setLeftPower(0);
        indexer.setRightPower(0);
    }

    public double getLeftPower() {
        return inputs.leftPower;
    }

    public double getRightPower() {
        return inputs.rightPower;
    }
}
