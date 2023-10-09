package com.team2383.robot.autos;

import com.team2383.robot.autos.auto_blocks.Engage;
import com.team2383.robot.autos.auto_blocks.FullAutoCommand;
import com.team2383.robot.autos.auto_blocks.ScoreHighCommand;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class EngageAuto extends SequentialCommandGroup {
    public EngageAuto(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, WristSubsystem wrist,
            FeederSubsystem feeder, String pathName, boolean isCube) {

        addCommands(
                new ScoreHighCommand(elevator, wrist, feeder, isCube),
                new FullAutoCommand(drivetrain, elevator, wrist, feeder, pathName),
                new Engage(drivetrain, false));
    }
}
