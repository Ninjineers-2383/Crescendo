package com.team2383.robot.commands.subsystem.resting_hooks;

import java.util.function.DoubleSupplier;

import com.team2383.robot.subsystems.resting_hooks.RestingHookSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class RestingHooksPowerCommand extends Command {
    private final RestingHookSubsystem restingHooks;
    private final DoubleSupplier powerBoth;
    private final DoubleSupplier powerOne;

    public RestingHooksPowerCommand(RestingHookSubsystem restingHooks, DoubleSupplier powerBoth,
            DoubleSupplier powerOne) {
        this.restingHooks = restingHooks;
        this.powerBoth = powerBoth;
        this.powerOne = powerOne;

        addRequirements(restingHooks);
    }

    @Override
    public void execute() {
        restingHooks.setPowerBoth(powerBoth.getAsDouble());
        restingHooks.setPowerOne(powerBoth.getAsDouble() + powerOne.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        restingHooks.setPowerBoth(0);
    }

    @Override
    public boolean isFinished() {
        // return restingHooks.isDone();
        return false;
    }
}
