package com.team2383.robot.commands.subsystem.pivot;

import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class PivotZeroCommand extends ConditionalCommand {
    public PivotZeroCommand(PivotSubsystem pivot) {
        super(new PivotPositionCommand(pivot,
                PivotPresets.ZERO),
                new PivotPositionCommand(pivot,
                        PivotPresets.ZERO_BACK),
                () -> pivot.getAngle().getDegrees() < 90);
    }
}
