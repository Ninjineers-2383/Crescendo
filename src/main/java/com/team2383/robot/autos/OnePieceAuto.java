package com.team2383.robot.autos;

import com.team2383.robot.autos.AutoQuestionResponses.QuestionResponses;
import com.team2383.robot.autos.auto_blocks.PathfindCommand;
import com.team2383.robot.autos.auto_blocks.ScoreCommand;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OnePieceAuto extends SequentialCommandGroup {
    public OnePieceAuto(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, WristSubsystem wrist,
            FeederSubsystem feeder, QuestionResponses[] responses) {
        addCommands(
                new InstantCommand(
                        () -> drivetrain.forceOdometry(AutoStartPoses.transformPoseForAlliance(
                                AutoStartPoses.responseToPose(responses[0]), DriverStation.getAlliance().get())),
                        drivetrain),

                new ScoreCommand(elevator, wrist, feeder, responses[0], responses[1]),

                new PathfindCommand(responses[2]),
                new PathfindCommand(responses[3]),
                new PathfindCommand(responses[4]));
    }

}
