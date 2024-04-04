package com.team2383.robot.subsystems.drivetrain.SLAM;

import edu.wpi.first.math.geometry.Pose3d;

public record SLAMUpdate(Pose3d pose, Pose3d[] landmarks, long seenLandmarks, boolean newValue) {
}
