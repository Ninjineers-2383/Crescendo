package com.team2383.robot.commands.subsystem.trap_arm;

import com.team2383.robot.subsystems.trap_arm.TrapArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class TrapArmPositionCommand extends Command {
    private final TrapArmSubsystem trapArm;
    private final DoubleSupplier angle;
    private final boolean finish;
    private double previousAngle = 0;

    public TrapArmPositionCommand(TrapArmSubsystem trapArm, double angle) {
        this(trapArm, () -> angle, true);
    }

    public TrapArmPositionCommand(TrapArmSubsystem trapArm, DoubleSupplier angle) {
        this(trapArm, angle, false);
    }

    public TrapArmPositionCommand(TrapArmSubsystem trapArm, DoubleSupplier angle, boolean finish) {
        this.trapArm = trapArm;
        this.angle = angle;
        this.finish = finish;

        addRequirements(trapArm);
    }

    @Override
    public void execute() {
        if (angle.getAsDouble() != previousAngle) {
            trapArm.setAngle(angle.getAsDouble());
            previousAngle = angle.getAsDouble();
        }
    }

    @Override
    public boolean isFinished() {
        return finish && trapArm.isDone();
    }
}
