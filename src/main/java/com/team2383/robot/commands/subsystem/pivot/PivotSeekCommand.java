package com.team2383.robot.commands.subsystem.pivot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.team2383.robot.FieldConstants;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotSeekCommand extends Command {
    private final PivotSubsystem pivot;
    private final Supplier<Pose3d> poseSupplier;

    public PivotSeekCommand(PivotSubsystem pivot, Supplier<Pose3d> poseSupplier) {
        this.pivot = pivot;
        this.poseSupplier = poseSupplier;

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        double distanceToSpeaker = poseSupplier.get().getTranslation().toTranslation2d()
                .getDistance(FieldConstants.getSpeakerLocation());

        Pose2d drivePose2d = poseSupplier.get().toPose2d();

        Rotation2d angleToSpeaker = new Rotation2d(
                Math.atan2(FieldConstants.getSpeakerLocation().getY() - drivePose2d.getY(),
                        FieldConstants.getSpeakerLocation().getX() - drivePose2d.getX()));

        double angle = 1.2 * Math.atan2(1.6, distanceToSpeaker);

        if (drivePose2d.getRotation().minus(angleToSpeaker).getDegrees() > 90) {
            angle = Math.PI - angle;
        } else if (drivePose2d.getRotation().minus(angleToSpeaker).getDegrees() < -90) {
            angle = Math.PI - angle;
        }

        Logger.recordOutput("Pivot/DistanceToSpeaker", distanceToSpeaker);

        pivot.setPosition(angle);
    }
}
