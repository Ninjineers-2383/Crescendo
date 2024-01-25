package com.team2383.robot.subsystems.sim_components;

import org.littletonrobotics.junction.Logger;

import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimComponents extends SubsystemBase {
    private final PivotSubsystem pivot;

    public SimComponents(PivotSubsystem pivot) {
        this.pivot = pivot;
    }

    @Override
    public void periodic() {
        Pose3d[] pose = new Pose3d[1];

        Pose3d basePose = new Pose3d(0, 0, 0.055,
                new Rotation3d(Units.degreesToRadians(90), Units.degreesToRadians(90), 0));

        Translation3d pivotCenterPose = new Translation3d(0.01, 0.0, 0.316);

        Rotation3d pivotRotationCenter = new Rotation3d(0, pivot.getAngle(), 0);

        Transform3d pivotPose = new Transform3d(pivotCenterPose, pivotRotationCenter);

        pose[0] = basePose.plus(pivotPose);

        Logger.recordOutput("Components", pose);
    }

}
