package com.team2383.robot.commands.feeding;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PPNeverEndCommand extends Command {
    private final Command command;

    public PPNeverEndCommand(Command command) {
        this.command = command;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(command);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
