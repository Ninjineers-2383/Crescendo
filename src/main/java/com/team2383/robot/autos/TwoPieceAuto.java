package com.team2383.robot.autos;

import com.team2383.robot.autos.auto_blocks.AutoStartPoses;
import com.team2383.robot.autos.auto_blocks.PathfindCommand;
import com.team2383.robot.autos.auto_blocks.ScoreCommand;
import com.team2383.robot.autos.auto_chooser.AutoQuestionResponses.QuestionResponses;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoPieceAuto extends SequentialCommandGroup {
    public TwoPieceAuto(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, WristSubsystem wrist,
            FeederSubsystem feeder, QuestionResponses[] responses) {
        addCommands(
                // Force odometry to the start pose of the auto
                new InstantCommand(
                        () -> drivetrain.forceOdometry(
                                AutoStartPoses.responseToPose(responses[0]), true),
                        drivetrain),

                // Score at the specified level using the specified piece
                new ScoreCommand(elevator, wrist, feeder, responses[0], responses[1]),

                // Either stop, engage in the community, mobility clean, or mobility dirty
                new PathfindCommand(drivetrain, responses[2]),

                // Feed the specified piece
                new PathfindCommand(drivetrain, responses[3]),

                // Pathfind to score the game piece
                new PathfindCommand(AutoStartPoses.responseToPose(responses[4]), 0),

                // Score the game piece
                new ScoreCommand(elevator, wrist, feeder, responses[4], responses[5]),

                // Either stop, or engage in the community
                new PathfindCommand(drivetrain, responses[6]));
    }

}
