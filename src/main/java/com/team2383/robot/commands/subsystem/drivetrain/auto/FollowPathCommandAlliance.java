package com.team2383.robot.commands.subsystem.drivetrain.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team2383.lib.util.AllianceUtil;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class FollowPathCommandAlliance extends Command {

    private String pathString;

    private Command command;

    public FollowPathCommandAlliance(DrivetrainSubsystem drivetrain, String pathString) {
        this.pathString = pathString;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        command = AutoBuilder.followPath(AllianceUtil.flipPath(PathPlannerPath.fromPathFile(pathString)));

        command.initialize();

    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
