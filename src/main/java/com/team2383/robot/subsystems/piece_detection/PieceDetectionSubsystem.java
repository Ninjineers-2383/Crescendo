package com.team2383.robot.subsystems.piece_detection;

import org.littletonrobotics.junction.Logger;

import com.team2383.lib.util.mechanical_advantage.Alert;
import com.team2383.lib.util.mechanical_advantage.Alert.AlertType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PieceDetectionSubsystem extends SubsystemBase {
    public PieceDetectionIO io;
    public PieceDetectionIOInputsAutoLogged inputs = new PieceDetectionIOInputsAutoLogged();

    private final Alert frontCameraDisconnected = new Alert("Piece Detection Limelight Disconnected!",
            AlertType.WARNING);

    public PieceDetectionSubsystem(PieceDetectionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("PieceDetection", inputs);

        frontCameraDisconnected.set(!inputs.connected);

    }

}
