package com.team2383.robot.subsystems.piece_detection;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PieceDetectionIOPhoton implements PieceDetectionIO {
    private final PhotonCamera frontCamera;

    public PieceDetectionIOPhoton() {
        frontCamera = new PhotonCamera("Camera_Module_Front");
    }

    @Override
    public void updateInputs(PieceDetectionIOInputs inputs) {
        inputs.connected = frontCamera.isConnected();

        PhotonPipelineResult frontResult = frontCamera.getLatestResult();

        if (frontResult.hasTargets()) {
            inputs.frontSeesTarget = true;
            inputs.frontYaw = frontResult.getBestTarget().getYaw();
            inputs.frontPitch = frontResult.getBestTarget().getPitch();
        } else {
            inputs.frontSeesTarget = false;
            inputs.frontYaw = 0.0;
            inputs.frontPitch = 0.0;
        }
    }
}
