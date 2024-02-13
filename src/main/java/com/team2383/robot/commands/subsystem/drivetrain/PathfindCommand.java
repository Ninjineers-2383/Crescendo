package com.team2383.robot.commands.subsystem.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class PathfindCommand extends Command {
    private Supplier<Pose2d> pose;

    private Command command;

    public PathfindCommand(DrivetrainSubsystem drivetrain, Supplier<Pose2d> pose) {
        this.pose = pose;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        command = AutoBuilder.pathfindToPose(pose.get(), DriveConstants.AUTO_CONSTRAINTS);

        command.initialize();
        ;
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
