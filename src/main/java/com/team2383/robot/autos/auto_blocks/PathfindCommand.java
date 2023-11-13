package com.team2383.robot.autos.auto_blocks;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team2383.robot.autos.auto_chooser.AutoQuestionResponses.QuestionResponses;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathfindCommand extends SequentialCommandGroup {
    public PathfindCommand(String pathName) {
        addCommands(AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile(pathName),
                DriveConstants.AUTO_CONSTRAINTS, 0));
    }

    public PathfindCommand(DrivetrainSubsystem drivetrain, QuestionResponses response) {
        switch (response) {
            case ENGAGECOMMUNITY:
                addCommands(
                        AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("EngageCommunity"),
                                DriveConstants.AUTO_CONSTRAINTS,
                                0),
                        new Engage(drivetrain, true));
                break;
            case ENGAGEOUTSIDE:
                addCommands(
                        AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("EngageOutside"),
                                DriveConstants.AUTO_CONSTRAINTS,
                                0),
                        new Engage(drivetrain, true));
                break;
            case MOBILITYCLEAN:
                addCommands(AutoBuilder.pathfindToPose(
                        AutoStartPoses.MOBILITYCLEAN,
                        DriveConstants.AUTO_CONSTRAINTS,
                        1,
                        0));
                break;
            case MOBILITYDIRTY:
                addCommands(AutoBuilder.pathfindToPose(
                        AutoStartPoses.MOBILITYDIRTY,
                        DriveConstants.AUTO_CONSTRAINTS,
                        1,
                        0));
                break;
            case MOBILITYCHARGE:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("MobilityCharge"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case STOP:
                addCommands(new InstantCommand());
                break;
            case FEEDCONECLEAN1:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("CleanFeedCone1"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCUBECLEAN1:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("CleanFeedCube1"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCONECLEAN2:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("CleanFeedCone2"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCUBECLEAN2:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("CleanFeedCube2"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCONEDIRTY1:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("DirtyFeedCone1"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCUBEDIRTY1:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("DirtyFeedCube1"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCONEDIRTY2:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("DirtyFeedCone2"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCUBEDIRTY2:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("DirtyFeedCube2"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCONECHARGE1:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("ChargeFeedCone1"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCUBECHARGE1:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("ChargeFeedCube1"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCONECHARGE2:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("ChargeFeedCone2"),
                        DriveConstants.AUTO_CONSTRAINTS,
                        0));
                break;
            case FEEDCUBECHARGE2:
                addCommands(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("ChargeFeedCube2"),
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
