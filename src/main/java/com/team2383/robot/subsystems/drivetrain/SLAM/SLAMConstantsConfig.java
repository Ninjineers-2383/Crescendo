package com.team2383.robot.subsystems.drivetrain.SLAM;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class SLAMConstantsConfig {
    // Description of the camera's relative position
    private static final Rotation3d CAM_ROTATION_FL = new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-20));
    private static final Rotation3d CAM_ROTATION_FR = new Rotation3d(0, Math.toRadians(-40), Math.toRadians(-0));
    private static final Rotation3d CAM_ROTATION_BL = new Rotation3d(0, Math.toRadians(-40), Math.toRadians(-180));
    private static final Rotation3d CAM_ROTATION_BR = new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-200));

    public static final double POSE_VARIANCE_SCALE = 0.87;
    public static final double POSE_VARIANCE_STATIC = 0.1;

    private static final Translation3d CAM_FRONT_LEFT = new Translation3d(
            0.267,
            0.281,
            0.211);

    private static final Translation3d CAM_FRONT_RIGHT = new Translation3d(
            0.274,
            -0.277,
            0.219);

    private static final Translation3d CAM_BACK_LEFT = new Translation3d(
            -0.274,
            0.277,
            0.219);

    private static final Translation3d CAM_BACK_RIGHT = new Translation3d(
            -0.267,
            -0.281,
            0.211);

    public static final Transform3d[] camTransforms = new Transform3d[] {
            new Transform3d(CAM_FRONT_LEFT, CAM_ROTATION_FL),
            new Transform3d(CAM_FRONT_RIGHT, CAM_ROTATION_FR),
            new Transform3d(CAM_BACK_LEFT, CAM_ROTATION_BL),
            new Transform3d(CAM_BACK_RIGHT, CAM_ROTATION_BR)
    };
}
