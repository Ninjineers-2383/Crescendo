package com.team2383.robot.subsystems.piece_detection;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PieceDetectionSubsystem extends SubsystemBase {
    public PieceDetectionIO io;
    public PieceDetectionIOInputsAutoLogged inputs = new PieceDetectionIOInputsAutoLogged();

    public PieceDetectionSubsystem(PieceDetectionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("PieceDetection", inputs);

    }

}
