package com.team2383.robot.subsystems.drivetrain.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {

    // Description of the camera's relative position
    private static final Rotation3d CAM_PITCH = new Rotation3d(0, Units.degreesToRadians(20), 0);
    private static final Rotation3d BACK_YAW = new Rotation3d(0, 0, Units.degreesToRadians(180));

    public static final double POSE_VARIANCE_SCALE = 1.2;
    public static final double POSE_VARIANCE_STATIC = 1E-9;

    private static final Translation3d CAM_FRONT_RIGHT = new Translation3d(
            Units.inchesToMeters(3.75 / 2),
            Units.inchesToMeters(12.25 / 2),
            Units.inchesToMeters(45));
    private static final Translation3d CAM_FRONT_LEFT = new Translation3d(
            Units.inchesToMeters(3.75 / 2),
            -Units.inchesToMeters(12.25 / 2),
            Units.inchesToMeters(45));
    private static final Translation3d CAM_REAR = new Translation3d(
            -Units.inchesToMeters(3.75 / 2),
            0,
            Units.inchesToMeters(45));

    public static final class PhotonCameraData {
        public final String name;
        public final Transform3d transform;

        public PhotonCameraData(String name, Transform3d transform) {
            this.name = name;
            this.transform = transform;
        }
    }

    public static final PhotonCameraData[] kPhotonCameras = new PhotonCameraData[] {
            new PhotonCameraData("Back",
                    new Transform3d(
                            CAM_REAR,
                            BACK_YAW.rotateBy(CAM_PITCH))),
            new PhotonCameraData("Front_Right",
                    new Transform3d(
                            CAM_FRONT_RIGHT,
                            CAM_PITCH)),
            new PhotonCameraData("Front_Left",
                    new Transform3d(
                            CAM_FRONT_LEFT,
                            CAM_PITCH))
    };

}