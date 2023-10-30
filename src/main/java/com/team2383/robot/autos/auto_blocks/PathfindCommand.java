package com.team2383.robot.autos.auto_blocks;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team2383.robot.autos.AutoStartPoses;
import com.team2383.robot.autos.AutoQuestionResponses.QuestionResponses;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathfindCommand extends SequentialCommandGroup {
    public PathfindCommand(String pathName) {
        addCommands(AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile(pathName),
                DriveConstants.AUTO_CONSTRAINTS, 0));
    }

    public PathfindCommand(QuestionResponses response) {
        switch (response) {
            case ENGAGECOMMUNITY:
                addCommands(
                        AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("EngageCommunity"),
                                DriveConstants.AUTO_CONSTRAINTS,
                                0));
                break;
            case ENGAGEOUTSIDE:
                addCommands(
                        AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("EngageOutside"),
                                DriveConstants.AUTO_CONSTRAINTS,
                                0));
                break;
            case MOBILITYCLEAN:
                addCommands(AutoBuilder.pathfindToPose(
                        AutoStartPoses.MOBILITYCLEAN,
                        DriveConstants.AUTO_CONSTRAINTS,
                        3,
                        0));
                break;
            case MOBILITYDIRTY:
                addCommands(AutoBuilder.pathfindToPose(
                        AutoStartPoses.MOBILITYDIRTY,
                        DriveConstants.AUTO_CONSTRAINTS,
                        3,
                        0));
                break;
            case STOP:
                addCommands(new InstantCommand());
                break;
            case FEEDCONECLEAN:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("CleanFeedCone"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCUBECLEAN:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("CleanFeedCube"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCONEDIRTY:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("DirtyFeedCone"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCUBEDIRTY:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("DirtyFeedCube"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            default:
                break;
        }
    }

    public PathfindCommand(Pose2d pose, double endVelocity) {
        addCommands(AutoBuilder.pathfindToPose(
                pose,
                DriveConstants.AUTO_CONSTRAINTS,
                endVelocity,
                0));
    }
}
