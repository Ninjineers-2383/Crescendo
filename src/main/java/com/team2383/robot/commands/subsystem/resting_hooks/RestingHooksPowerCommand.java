package com.team2383.robot.commands.subsystem.resting_hooks;

import com.team2383.robot.subsystems.resting_hooks.RestingHookSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class RestingHooksPowerCommand extends Command {
    private final RestingHookSubsystem restingHooks;

    public RestingHooksPowerCommand(RestingHookSubsystem restingHooks) {
        this.restingHooks = restingHooks;

        addRequirements(restingHooks);
    }

    @Override
    public void execute() {
        restingHooks.setPower(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        restingHooks.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return restingHooks.isDone();
    }
}
