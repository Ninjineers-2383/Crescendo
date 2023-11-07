package com.team2383.robot.autos.auto_blocks;

import com.team2383.robot.autos.auto_chooser.AutoQuestionResponses.QuestionResponses;
import com.team2383.robot.commands.FeederVoltageCommand;
import com.team2383.robot.commands.blizzard.BlizzardCommand;
import com.team2383.robot.commands.blizzard.BlizzardPresets;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederConstants;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreCommand extends SequentialCommandGroup {
    public ScoreCommand(ElevatorSubsystem elevator, WristSubsystem wrist, FeederSubsystem feeder,
            QuestionResponses gamePiece, QuestionResponses scoreLevel) {
        addCommands(
                new FeederVoltageCommand(feeder, () -> 0, FeederConstants.responseToGamePiece(gamePiece))
                        .withTimeout(0.1),
                new BlizzardCommand(elevator, wrist, BlizzardPresets.responseToPreset(scoreLevel)).withTimeout(1),
                new FeederVoltageCommand(feeder, () -> 0.75, FeederConstants.responseToGamePiece(gamePiece))
                        .withTimeout(0.1),
                new BlizzardCommand(elevator, wrist, BlizzardPresets.CONE_CHUTE).withTimeout(0.5),
                new FeederVoltageCommand(feeder, () -> 0, FeederConstants.responseToGamePiece(gamePiece))
                        .withTimeout(0.1));
    }
}
