package com.team2383.robot.commands.orchestra;

import com.team2383.robot.subsystems.orchestra.OrchestraContainer;

import edu.wpi.first.wpilibj2.command.Command;

public class OrchestraCommand extends Command {
    private OrchestraContainer orchestra = OrchestraContainer.getInstance();

    private final String fileName;

    public OrchestraCommand(String fileName) {
        this.fileName = fileName;
    }

    @Override
    public void initialize() {
        orchestra.loadMusic(fileName);
    }

    @Override
    public void execute() {
        orchestra.play();
    }

    @Override
    public void end(boolean interrupted) {
        orchestra.pause();
    }

}
