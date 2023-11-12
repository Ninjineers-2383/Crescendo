package com.team2383.robot.autos.auto_blocks;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team2383.robot.autos.auto_chooser.AutoQuestionResponses.QuestionResponses;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, WristSubsystem wrist,
            FeederSubsystem feeder,
            Supplier<QuestionResponses[]> responses) {

        Command scoreCommand = new SequentialCommandGroup(AutoBuilder.pathfindToPose(
                AutoStartPoses.responseToPose(responses.get()[0]),
                DriveConstants.AUTO_CONSTRAINTS,
                0).andThen(new ScoreCommand(elevator, wrist, feeder, responses.get()[0], responses.get()[1])));

        addCommands(scoreCommand);
    }
}
