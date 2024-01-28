package com.team2383.robot.subsystems.sim_components;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimComponents extends SubsystemBase {
    private final PivotSubsystem pivot;

    public SimComponents(PivotSubsystem pivot) {
        this.pivot = pivot;
    }

    @Override
    public void periodic() {
        Pose3d[] pose = new Pose3d[1];

        Rotation3d pivotRotation = new Rotation3d(pivot.getAngle(), 0, 0);
        Rotation3d robotRotation = new Rotation3d(Math.toRadians(90), 0, Math.toRadians(90));
        Pose3d pivotPose = new Pose3d(0, 0, 0.055,
                pivotRotation.plus(robotRotation));

        pose[0] = pivotPose;

        Logger.recordOutput("Components", pose);
    }

}
