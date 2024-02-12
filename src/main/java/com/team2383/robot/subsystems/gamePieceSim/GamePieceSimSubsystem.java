package com.team2383.robot.subsystems.gamePieceSim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import org.littletonrobotics.junction.Logger;

public class GamePieceSimSubsystem extends SubsystemBase {
    private final Supplier<Pose3d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private BooleanSupplier shootingSupplier;
    private BooleanSupplier intakeFrontSupplier;
    private BooleanSupplier intakeRearSupplier;
    private DoubleSupplier shooterAngle;
    private DoubleSupplier shooterRPM;

    private int shooterCounter = 0;

    private ChassisSpeeds initialShootingSpeeds = new ChassisSpeeds();

    private Pose3d[] notes = GamePieceLocations.notes;
    private boolean[] notesHit = new boolean[notes.length];

    public GamePieceSimSubsystem(Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier,
            BooleanSupplier shootingSupplier, BooleanSupplier intakeFrontSupplier, BooleanSupplier intakeRearSupplier,
            DoubleSupplier shooterAngle,
            DoubleSupplier shooterRPM) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.shootingSupplier = shootingSupplier;
        this.intakeFrontSupplier = intakeFrontSupplier;
        this.intakeRearSupplier = intakeRearSupplier;
        this.shooterAngle = shooterAngle;
        this.shooterRPM = shooterRPM;
    }

    @Override
    public void periodic() {
        Pose3d pose = poseSupplier.get();

        boolean intakeFront = intakeFrontSupplier.getAsBoolean();
        boolean intakeRear = intakeRearSupplier.getAsBoolean();

        boolean shooting = shootingSupplier.getAsBoolean();
        ChassisSpeeds speeds = speedsSupplier.get();
        Rotation2d angle = Rotation2d.fromDegrees(shooterAngle.getAsDouble());
        double RPM = shooterRPM.getAsDouble();

        for (int i = 0; i < notes.length; i++) {
            if (pose.getTranslation().getDistance(notes[i].getTranslation()) < 0.5
                    && notesHit[i] == false) {
                int collisionSide = getCollisionSide(pose, notes[i]);

                if ((intakeFront || intakeRear) && !pieceInRobot()) {
                    if (collisionSide == 1 || collisionSide == 4) {
                        notes[i] = new Pose3d(new Translation3d(pose.getX(), pose.getY(), 1),
                                new Rotation3d());

                        notesHit[i] = true;
                    } else {
                        notesHit[i] = false;
                    }
                }

                movePiece(i, collisionSide, pose);
            }

            if (notesHit[i] == true) {
                if (getNoteFieldRelativeZ(i, angle) < 0) {
                    notesHit[i] = false;
                    notes[i] = new Pose3d(new Translation3d(notes[i].getX(), notes[i].getY(), 0),
                            new Rotation3d(Math.toRadians(-90), 0, 0).plus(pose.getRotation()));

                    shooterCounter = 0;
                } else if (shooting) {
                    if (shooterCounter == 0) {
                        initialShootingSpeeds = speeds;
                        Logger.recordOutput("Score", false);

                    }

                    Translation3d noteTranslation = getNoteFieldRelativeTranslation(i, angle);

                    if (noteTranslation.getX() < 0 && noteTranslation.getY() < 6 && noteTranslation.getY() > 5
                            && noteTranslation.getZ() > 2 && noteTranslation.getZ() < 2.5) {
                        Logger.recordOutput("Score", true);
                    }

                    notes[i] = notes[i].exp(shoot(initialShootingSpeeds, angle, RPM, shooterCounter));

                    shooterCounter++;
                } else {
                    notes[i] = new Pose3d(new Translation3d(pose.getX(), pose.getY(), 0.2),
                            new Rotation3d(0, Math.toRadians(-90) - angle.getRadians(), 0)
                                    .rotateBy(pose.getRotation()));

                    shooterCounter = 0;
                }
            }

            Logger.recordOutput("Notes", notes);
        }
    }

    public boolean pieceInRobot() {
        for (boolean notes : notesHit) {
            if (notes) {
                return true;
            }
        }

        return false;
    }

    public void movePiece(int noteIndex, int collisionSide, Pose3d robotPose) {
        ChassisSpeeds speeds = speedsSupplier.get();

        Rotation2d speedsAngle = new Rotation2d(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond))
                .plus(new Rotation2d(Math.PI)).plus(robotPose.getRotation().toRotation2d());

        double speedsAngleDegrees = speedsAngle.getDegrees();

        boolean move = false;
        switch (collisionSide) {
            case 1:
                if (speedsAngleDegrees > -80 && speedsAngleDegrees < 80) {
                    move = true;
                }
                break;
            case 2:
                if (speedsAngleDegrees > 10 && speedsAngleDegrees < 170) {
                    move = true;
                }
                break;
            case 3:
                if (speedsAngleDegrees < -10 && speedsAngleDegrees > -170) {
                    move = true;
                }
                break;
            case 4:
                if (speedsAngleDegrees > 80 || speedsAngleDegrees < -80) {
                    move = true;
                }
                break;
            default:
                break;

        }
        if (move) {
            notes[noteIndex] = notes[noteIndex]
                    .exp(new Twist3d(speeds.vxMetersPerSecond * 0.02, speeds.vyMetersPerSecond * 0.02, 0, 0, 0, 0));
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

        return side;
    }

    public Translation3d getNoteFieldRelativeTranslation(int noteIndex, Rotation2d shooterAngle) {
        return new Translation3d(notes[noteIndex].getX() * shooterAngle.getSin(), notes[noteIndex].getY(),
                notes[noteIndex].getZ() * shooterAngle.getSin());
    }

    public double getNoteFieldRelativeZ(int noteIndex, Rotation2d shooterAngle) {
        return notes[noteIndex].getZ() * shooterAngle.getSin();
    }

    public double getNoteFieldRelativeX(int noteIndex, Rotation2d shooterAngle) {
        return notes[noteIndex].getX() * shooterAngle.getSin();
    }

    public double getNoteFieldRelativeY(int noteIndex, Rotation2d shooterAngle) {
        return notes[noteIndex].getY();
    }

    public Twist3d shoot(ChassisSpeeds robotSpeed, Rotation2d shooterAngle, double shooterRPM, int counter) {
        double time = counter * 0.02;

        double shooterLinearSpeed = (shooterRPM / 60) * 2 * Math.PI * GamePieceLocations.shooterRadiusMeters;

        double initialVeloX = (shooterLinearSpeed * shooterAngle.getCos()) * (1 - GamePieceLocations.coefficientOfDrag);

        double initialVeloY = shooterLinearSpeed * shooterAngle.getSin();

        double veloY = (-9.8 * time + initialVeloY) / (1 + GamePieceLocations.coefficientOfDrag);

        return new Twist3d(shooterAngle.getCos() * (veloY
                - (initialVeloX + robotSpeed.vxMetersPerSecond))
                * 0.02,
                robotSpeed.vyMetersPerSecond * 0.02,
                -shooterAngle.getSin() * (veloY + initialVeloX + robotSpeed.vxMetersPerSecond) * 0.02, 0, 0, 0);
    }

    public boolean hasScored() {
        return false;
    }

    public void movePiece(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed, IntSupplier noteIndex) {
        notes[noteIndex.getAsInt()] = notes[noteIndex.getAsInt()].exp(new Twist3d(zSpeed.getAsDouble() * 0.1,
                ySpeed.getAsDouble() * 0.1, -xSpeed.getAsDouble() * 0.1, 0, 0, 0));

    }
}
