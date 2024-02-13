package com.team2383.robot.commands.subsystem.orchestra;

import com.team2383.robot.subsystems.orchestra.OrchestraContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class OrchestraCommand extends Command {
    private OrchestraContainer orchestra = OrchestraContainer.getInstance();

    private final String fileName;

    public OrchestraCommand(String fileName, Subsystem... requirements) {
        this.fileName = fileName;
        addRequirements(requirements);
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
