package com.team2383.robot.subsystems.sim_components;

import org.littletonrobotics.junction.Logger;

import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimComponents extends SubsystemBase {
    private final ElevatorSubsystem elevator;

    public SimComponents(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    @Override
    public void periodic() {

        Pose3d basePose = new Pose3d(0, 0, 0.1, new Rotation3d(Units.degreesToRadians(90), 0, 0));

        Pose3d[] poses = new Pose3d[3];
        Rotation3d elevatorRotation = new Rotation3d(0, 0, -Units.degreesToRadians(35));
        Translation3d elevatorTop = new Translation3d(0, this.elevator.getPosition(), 0).rotateBy(elevatorRotation);
        Translation3d elevatorBottom = new Translation3d(0, this.elevator.getPosition() / 2.0, 0)
                .rotateBy(elevatorRotation);
        poses[0] = basePose;
        poses[1] = basePose.plus(new Transform3d(elevatorBottom, new Rotation3d(0, 0, 0)));
        poses[2] = basePose.plus(new Transform3d(elevatorTop, new Rotation3d(0, 0, 0)));

        Logger.getInstance().recordOutput("Components", poses);
    }

}
