package com.team2383.robot.subsystems.gamePieceSim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
                int collisionSide = getCollisionSide(pose, notes[i]);

                movePiece(i, collisionSide);

            }
        }

        Logger.recordOutput("Notes", notes);
    }

    public void movePiece(int noteIndex, int collisionSide) {
        ChassisSpeeds speeds = speedsSupplier.get();

        Rotation2d speedsAngle = new Rotation2d(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond))
                .plus(new Rotation2d(Math.PI));

        double speedsAngleDegrees = speedsAngle.getDegrees();

        boolean move = false;
        switch (collisionSide) {
            case 1:
                if (speedsAngleDegrees > -45 && speedsAngleDegrees < 45) {
                    move = true;
                }
                break;
            case 2:
                if (speedsAngleDegrees > 45 && speedsAngleDegrees < 135) {
                    move = true;
                }
                break;
            case 3:
                if (speedsAngleDegrees < -45 && speedsAngleDegrees > -135) {
                    move = true;
                }
                break;
            case 4:
                if (speedsAngleDegrees > 135 || speedsAngleDegrees < -135) {
                    move = true;
                }
                break;
            default:
                break;

        }
        if (move) {
            notes[noteIndex] = notes[noteIndex]
                    .exp(new Twist3d(0, speeds.vyMetersPerSecond * 0.04, -speeds.vxMetersPerSecond * 0.04, 0, 0, 0));
        }

        Logger.recordOutput("Speeds Angle", speedsAngleDegrees);

    }

    public Rotation2d getCollisionAngle(Pose3d robotPose, Pose3d notePose) {
        return new Rotation2d(Math.atan2(robotPose.getY() - notePose.getY(), robotPose.getX() - notePose.getX()))
                .plus(robotPose.getRotation().toRotation2d());
    }

    public int getCollisionSide(Pose3d robotPose, Pose3d notePose) {
        Rotation2d collisionAngle = getCollisionAngle(robotPose, notePose);
        double collisionAngleDegrees = collisionAngle.getDegrees();
        int side = 0;

        if (collisionAngleDegrees > -45 && collisionAngleDegrees < 45) {
            side = 1;
        } else if (collisionAngleDegrees > 45 && collisionAngleDegrees < 135) {
            side = 2;
        } else if (collisionAngleDegrees < -45 && collisionAngleDegrees > -135) {
            side = 3;
        } else if (collisionAngleDegrees > 135 || collisionAngleDegrees < -135) {
            side = 4;
        }

        Logger.recordOutput("Collision Side", side);
        Logger.recordOutput("Collision Angle", collisionAngleDegrees);
        return side;
    }
}
