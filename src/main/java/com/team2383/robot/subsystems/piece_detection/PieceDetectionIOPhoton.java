package com.team2383.robot.subsystems.piece_detection;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PieceDetectionIOPhoton implements PieceDetectionIO {
    private final PhotonCamera photonCamera;

    public PieceDetectionIOPhoton() {
        photonCamera = new PhotonCamera("Camera_Module_v1");
    }

    @Override
    public void updateInputs(PieceDetectionIOInputs inputs) {
        inputs.connected = true;

        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            inputs.seesTarget = true;
            inputs.yaw = result.getBestTarget().getYaw();
        } else {
            inputs.seesTarget = false;
            inputs.yaw = 0.0;
        }
    }
}
