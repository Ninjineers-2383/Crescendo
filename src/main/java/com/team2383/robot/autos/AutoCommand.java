package com.team2383.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoCommand extends SequentialCommandGroup {
    public AutoCommand(Pose2d startPose, DrivetrainSubsystem drivetrain) {
        addCommands(
                new InstantCommand(() -> drivetrain.forceOdometry(startPose), drivetrain),
                AutoBuilder.pathfindToPose(
                        new Pose2d(10, 5, Rotation2d.fromDegrees(180)),
                        new PathConstraints(
                                3.0, 4.0,
                                Units.degreesToRadians(540), Units.degreesToRadians(720)),
                        0.0, // Goal end velocity in meters/sec
                        0.0 // Rotation delay distance in meters. This is how far the robot should travel
                            // before attempting to rotate.
                ));
    }

}
