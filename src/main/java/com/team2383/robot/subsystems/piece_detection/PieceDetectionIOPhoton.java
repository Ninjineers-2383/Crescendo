package com.team2383.robot.subsystems.piece_detection;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PieceDetectionIOPhoton implements PieceDetectionIO {
    private final PhotonCamera frontCamera;
    private final PhotonCamera rearCamera;

    public PieceDetectionIOPhoton() {
        frontCamera = new PhotonCamera("Camera_Module_Front");
        rearCamera = new PhotonCamera("Camera_Module_Rear");
    }

    @Override
    public void updateInputs(PieceDetectionIOInputs inputs) {
        inputs.connected = true;

        PhotonPipelineResult frontResult = frontCamera.getLatestResult();
        PhotonPipelineResult rearResult = rearCamera.getLatestResult();

        if (frontResult.hasTargets()) {
            inputs.frontSeesTarget = true;
            inputs.frontYaw = frontResult.getBestTarget().getYaw();
        } else {
            inputs.frontSeesTarget = false;
            inputs.frontYaw = 0.0;
        }

        if (rearResult.hasTargets()) {
            inputs.rearSeesTarget = true;
            inputs.rearYaw = rearResult.getBestTarget().getYaw();
        } else {
            inputs.rearSeesTarget = false;
            inputs.rearYaw = 0.0;
        }
    }
}
