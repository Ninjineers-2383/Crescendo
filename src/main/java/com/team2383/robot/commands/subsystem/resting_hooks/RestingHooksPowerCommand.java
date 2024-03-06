package com.team2383.robot.commands.subsystem.resting_hooks;

import java.util.function.DoubleSupplier;

import com.team2383.robot.subsystems.resting_hooks.RestingHookSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class RestingHooksPowerCommand extends Command {
    private final RestingHookSubsystem restingHooks;
    private final DoubleSupplier power;

    public RestingHooksPowerCommand(RestingHookSubsystem restingHooks, DoubleSupplier power) {
        this.restingHooks = restingHooks;
        this.power = power;

        addRequirements(restingHooks);
    }

    @Override
    public void execute() {
        restingHooks.setPower(power.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        restingHooks.setPower(0);
    }

    @Override
    public boolean isFinished() {
        // return restingHooks.isDone();
        return false;
    }
}
