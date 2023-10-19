package com.team2383.robot.subsystems.drivetrain.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {

    // Description of the camera's relative position
    private static final Rotation3d CAM_ROTATION = new Rotation3d(0, 0, 0);

    public static final double POSE_VARIANCE_SCALE = 0.4;
    public static final double POSE_VARIANCE_STATIC = 1E-9;

    private static final Translation3d CAM_LEFT = new Translation3d(
            Units.inchesToMeters(8),
            -Units.inchesToMeters(12 / 2),
            Units.inchesToMeters(35));
    private static final Translation3d CAM_RIGHT = new Translation3d(
            -Units.inchesToMeters(8),
            Units.inchesToMeters(12 / 2),
            Units.inchesToMeters(35));

    public static final class PhotonCameraData {
        public final String name;
        public final Transform3d transform;

        public PhotonCameraData(String name, Transform3d transform) {
            this.name = name;
            this.transform = transform;
        }
    }

    public static final PhotonCameraData[] kPhotonCameras = new PhotonCameraData[] {
            new PhotonCameraData("Left",
                    new Transform3d(
                            CAM_LEFT,
                            CAM_ROTATION)),
            new PhotonCameraData("Right",
                    new Transform3d(
                            CAM_RIGHT,
                            CAM_ROTATION))
    };

}