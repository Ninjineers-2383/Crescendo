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
    private final boolean finish;

    public PivotSeekCommand(PivotSubsystem pivot, Supplier<Pose3d> poseSupplier, boolean finish) {
        this.pivot = pivot;
        this.poseSupplier = poseSupplier;
        this.finish = finish;

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

        // https://www.desmos.com/calculator/et7ibvp93g
        // 1.58175
        double angle = Math.atan2(1.9, distanceToSpeaker) + (0.008 * distanceToSpeaker * distanceToSpeaker);

        if (drivePose2d.getRotation().minus(angleToSpeaker).getDegrees() > 90) {
            angle = Math.PI - angle;
        } else if (drivePose2d.getRotation().minus(angleToSpeaker).getDegrees() < -90) {
            angle = Math.PI - angle;
        }

        Logger.recordOutput("Pivot/DistanceToSpeaker", distanceToSpeaker);

        pivot.setPosition(angle);
    }

    @Override
    public boolean isFinished() {
        if (finish) {
            return pivot.isFinished();
        }

        return false;
    }
}
