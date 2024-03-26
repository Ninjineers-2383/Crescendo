package com.team2383.robot.subsystems.gamePieceSim;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class GamePieceSim extends SubsystemBase {
    private final Supplier<Pose3d> m_robotPose;
    private final Supplier<ChassisSpeeds> m_robotRelativeSpeeds;
    private final Supplier<Rotation2d> m_pivotAngle;
    private final DoubleSupplier m_shooterRPM;
    private final BooleanSupplier m_fullFeed;
    private final BooleanSupplier m_partialFeed;
    private final DoubleSupplier m_indexerPower;

    private Pose3d[] notes = GamePieceLocations.notes;
    private boolean[] notesHit = new boolean[notes.length];
    private boolean[] notesPartialFed = new boolean[notes.length];
    private boolean[] notesFullyFed = new boolean[notes.length];
    private boolean[] notesShooting = new boolean[notes.length];

    private boolean feederBeamBreakTripped = false;
    private boolean indexerBeamBreakTripped = false;

    private int shootingCounter = 0;

    private int notesScored = 0;

    public GamePieceSim(Supplier<Pose3d> robotPose, Supplier<ChassisSpeeds> robotRelativeSpeeds,
            Supplier<Rotation2d> pivotAngle, DoubleSupplier shooterRPM,
            BooleanSupplier fullFeed, BooleanSupplier partialFeed, DoubleSupplier indexerPower) {
        m_robotPose = robotPose;
        m_robotRelativeSpeeds = robotRelativeSpeeds;
        m_pivotAngle = pivotAngle;
        m_shooterRPM = shooterRPM;
        m_fullFeed = fullFeed;
        m_partialFeed = partialFeed;
        m_indexerPower = indexerPower;
    }

    @Override
    public void periodic() {
        Pose3d pose = m_robotPose.get();
        ChassisSpeeds speeds = m_robotRelativeSpeeds.get();
        boolean fullFeed = m_fullFeed.getAsBoolean();
        Rotation2d angle = m_pivotAngle.get();
        double shooterRPM = m_shooterRPM.getAsDouble();
        boolean partialFeed = m_partialFeed.getAsBoolean();
        double indexerPower = m_indexerPower.getAsDouble();

        for (int i = 0; i < notes.length; i++) {
            if (pose.getTranslation().getDistance(notes[i].getTranslation()) < 0.5
                    && !notesHit[i]) {
                notesHit[i] = true;

                String collisionSide = getCollisionSide(pose, notes[i]);

                if (collisionSide == "Back") {
                    if (partialFeed) {
                        notesPartialFed[i] = true;
                        feederBeamBreakTripped = true;
                    }

                    if (fullFeed) {
                        notesFullyFed[i] = true;
                        indexerBeamBreakTripped = true;
                        feederBeamBreakTripped = false;
                    }

                }

                Logger.recordOutput("GamePieceSim/Note Hit", "Note " + i + " hit");
                Logger.recordOutput("GamePieceSim/Collision Side", collisionSide);
            } else {
                notesHit[i] = false;
            }

            if (notesPartialFed[i]) {
                notes[i] = pose.plus(new Transform3d(-0.145, 0, 0.197, new Rotation3d(0, Math.toRadians(90), 0)));
            }

            if (notesFullyFed[i]) {
                notes[i] = pose.plus(new Transform3d(0, 0, 0.471 + 0.055, new Rotation3d(0, -angle.getRadians(), 0)));

                if (indexerPower < -0.8) {
                    notesPartialFed[i] = false;
                    notesFullyFed[i] = false;

                    notesShooting[i] = true;
                    indexerBeamBreakTripped = false;
                }
            }

            if (notesShooting[i]) {
                if (notes[i].getTranslation().getZ() < 0) {
                    notesShooting[i] = false;
                    shootingCounter = 0;
                    notes[i] = new Pose3d(new Translation3d(notes[i].getTranslation().getX(),
                            notes[i].getTranslation().getY(), GamePieceLocations.noteHeight), new Rotation3d());
                } else {
                    notes[i] = notes[i].exp(shoot(speeds, angle, shooterRPM, shootingCounter));
                    shootingCounter++;
                    Translation3d noteTranslation = notes[i].getTranslation();

                    if (noteTranslation.getX() < 0 && noteTranslation.getY() < 6 && noteTranslation.getY() > 5
                            && noteTranslation.getZ() > 2 && noteTranslation.getZ() < 2.5) {
                        notesScored++;
                    }

                }
            }
        }

        Logger.recordOutput("GamePieceSim/Notes", notes);
        Logger.recordOutput("GamePieceSim/Indexer Beam Break Tripped", indexerBeamBreakTripped);
        Logger.recordOutput("GamePieceSim/Feeder Beam Break Tripped", feederBeamBreakTripped);
        Logger.recordOutput("GamePieceSim/Notes Scored", notesScored);
    }

    public String getCollisionSide(Pose3d robotPose, Pose3d notePose) {
        Rotation2d collisionAngle = getCollisionAngle(robotPose, notePose);
        double collisionAngleDegrees = collisionAngle.getDegrees();
        String side = "None";

        if (collisionAngleDegrees > -45 && collisionAngleDegrees < 45) {
            side = "Back";
        } else if (collisionAngleDegrees > 45 && collisionAngleDegrees < 135) {
            side = "Left";
        } else if (collisionAngleDegrees < -45 && collisionAngleDegrees > -135) {
            side = "Right";
        } else if (collisionAngleDegrees > 135 || collisionAngleDegrees < -135) {
            side = "Front";
        }

        return side;
    }

    public Rotation2d getCollisionAngle(Pose3d robotPose, Pose3d notePose) {
        return new Rotation2d(Math.atan2(robotPose.getY() - notePose.getY(), robotPose.getX() - notePose.getX()))
                .plus(robotPose.getRotation().toRotation2d());
    }

    public Twist3d shoot(ChassisSpeeds robotSpeed, Rotation2d shooterAngle, double shooterRPM, int counter) {
        double time = counter * 0.02;

        double shooterLinearSpeed = (shooterRPM / 60) * 2 * Math.PI *
                GamePieceLocations.shooterRadiusMeters;

        double veloY = (-9.8 * time) / (1 + GamePieceLocations.coefficientOfDrag);

        double pieceVeloX = shooterLinearSpeed - veloY * shooterAngle.getSin();
        double pieceVeloY = veloY * shooterAngle.getCos();

        return new Twist3d(pieceVeloX * 0.02,
                0,
                pieceVeloY * 0.02,
                0, 0, 0);
    }

    public boolean getFeederBeamBreakTripped() {
        return feederBeamBreakTripped;
    }

    public boolean getIndexerBeamBreakTripped() {
        return indexerBeamBreakTripped;
    }
}
