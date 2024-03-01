package com.team2383.robot.commands.speaker;

import org.littletonrobotics.junction.Logger;

import com.team2383.robot.commands.subsystem.drivetrain.FaceToSpeakerCommand;
import com.team2383.robot.commands.subsystem.pivot.PivotSeekCommand;
import com.team2383.robot.commands.subsystem.shooter.ShooterRPMCommand;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class SeekCommand extends ParallelCommandGroup {

    public SeekCommand(DrivetrainSubsystem drivetrain, PivotSubsystem pivot, ShooterSubsystem shooter, boolean finish) {

        addCommands(
                new FaceToSpeakerCommand(drivetrain, finish),
                new PivotSeekCommand(pivot, drivetrain::getEstimatorPose3d, finish),
                new Command() {
                    @Override
                    public final void initialize() {
                        Logger.recordOutput("Seeking/enabled", true);
                    }

                    @Override
                    public final void end(boolean interrupted) {
                        Logger.recordOutput("Seeking/enabled", false);

                    }
                });
        // new ShooterRPMCommand(shooter, () -> -5000, () -> 1000, () -> 0));
    }

}
