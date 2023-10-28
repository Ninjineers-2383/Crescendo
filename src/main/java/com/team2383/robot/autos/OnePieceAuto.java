package com.team2383.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team2383.robot.autos.AutoQuestionResponses.QuestionResponses;
import com.team2383.robot.autos.auto_blocks.Engage;
import com.team2383.robot.autos.auto_blocks.ScoreCommand;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OnePieceAuto extends SequentialCommandGroup {
    public OnePieceAuto(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, WristSubsystem wrist,
            FeederSubsystem feeder, QuestionResponses[] responses) {
        addCommands(
                new InstantCommand(() -> drivetrain.forceOdometry(AutoStartPoses.responseToPose(responses[0])),
                        drivetrain),

                new ScoreCommand(elevator, wrist, feeder, responses[0], responses[1]),

                new ConditionalCommand(
                        new InstantCommand(),
                        new SequentialCommandGroup(
                                AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Engage"),
                                        DriveConstants.AUTO_CONSTRAINTS, 0),
                                new Engage(drivetrain, true)),
                        () -> responses[2] == QuestionResponses.STOP));
    }

}
