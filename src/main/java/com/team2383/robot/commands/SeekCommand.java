package com.team2383.robot.commands;

import com.team2383.robot.commands.drivetrain.FaceToSpeakerCommand;
import com.team2383.robot.commands.shooter.ShooterRPMCommand;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class SeekCommand extends ParallelCommandGroup {
    // TODO: Add a command to pivot the shooter to the correct angle
    public SeekCommand(DrivetrainSubsystem drivetrain, PivotSubsystem pivot, ShooterSubsystem shooter) {

        addCommands(
                new FaceToSpeakerCommand(drivetrain),
                new ShooterRPMCommand(shooter, () -> 6800, () -> 1000, () -> 0));
    }

}
