package com.team2383.robot.subsystems.gamePieceSim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class GamePieceSimSubsystem extends SubsystemBase {
    private final Supplier<Pose3d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private BooleanSupplier shootingSupplier;

    private Pose3d[] notes = GamePieceLocations.notes;

    public GamePieceSimSubsystem(Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier,
            BooleanSupplier shootingSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.shootingSupplier = shootingSupplier;
    }

    @Override
    public void periodic() {
        Pose3d pose = poseSupplier.get();

        for (int i = 0; i < notes.length; i++) {
            if (pose.getTranslation().getDistance(notes[i].getTranslation()) < 0.5) {
                shoot(i);

            }
        }

        Logger.recordOutput("Notes", notes);
    }

    public void shoot(int noteIndex) {
        Pose3d pose = poseSupplier.get();

        ChassisSpeeds speeds = speedsSupplier.get();

        // Pose3d notePose = notes[noteIndex];

        notes[noteIndex] = notes[noteIndex]
                .exp(new Twist3d(0, speeds.vyMetersPerSecond * 0.04,
                        -speeds.vxMetersPerSecond * 0.04, 0, 0, 0));

        // notes[noteIndex] = notes[noteIndex]
        // .exp(new Twist3d(0, 0, 0.1, 0, 0, 0));
    }
}
