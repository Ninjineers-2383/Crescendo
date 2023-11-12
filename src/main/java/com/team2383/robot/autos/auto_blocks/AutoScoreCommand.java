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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreCommand extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final ElevatorSubsystem elevator;
    private final WristSubsystem wrist;
    private final FeederSubsystem feeder;
    private final Supplier<QuestionResponses[]> responses;

    public AutoScoreCommand(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, WristSubsystem wrist,
            FeederSubsystem feeder,
            Supplier<QuestionResponses[]> responses) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.wrist = wrist;
        this.feeder = feeder;
        this.responses = responses;
    }

    @Override
    public void initialize() {
        Command command = new SequentialCommandGroup(AutoBuilder.pathfindToPose(
                AutoStartPoses.responseToPose(responses.get()[0]), DriveConstants.AUTO_CONSTRAINTS, 0, 0),
                new ScoreCommand(elevator, wrist, feeder, responses.get()[0], responses.get()[1]));

        command.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        Command stopCommand = new InstantCommand(() -> {
        }, drivetrain, elevator, wrist, feeder);

        stopCommand.schedule();
    }
}
