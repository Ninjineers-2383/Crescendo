package com.team2383.robot.autos.auto_blocks;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import com.team2383.robot.subsystems.drivetrain.DriveConstants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathfindCommandTeleOp extends SequentialCommandGroup {
    public PathfindCommandTeleOp(String pathName) {
        addCommands(AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile(pathName),
                DriveConstants.AUTO_CONSTRAINTS, 0));
    }
}
