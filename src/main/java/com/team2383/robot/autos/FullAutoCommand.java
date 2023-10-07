package com.team2383.robot.autos;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullAutoCommand extends SequentialCommandGroup {
    public FullAutoCommand(DrivetrainSubsystem drivetrain, String pathName, SwerveAutoBuilder autoBuilder) {

        // This will load the file "FullAuto.path" and generate it with a max velocity
        // of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName,
                new PathConstraints(1.5, 0.7));
                
        addCommands(
                autoBuilder.fullAuto(pathGroup));
    }
}
