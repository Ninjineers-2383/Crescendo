package com.team2383.robot.commands.pivot;

import java.util.function.Supplier;

import com.team2383.robot.FieldConstants;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.math.geometry.Pose3d;
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

        double angle = distanceToSpeaker; // TODO: create formula

        pivot.setPosition(angle);
    }
}
