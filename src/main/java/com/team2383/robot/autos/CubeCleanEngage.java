package com.team2383.robot.autos;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CubeCleanEngage extends SequentialCommandGroup {
    public CubeCleanEngage(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, WristSubsystem wrist,
            FeederSubsystem feeder, String pathName, SwerveAutoBuilder autoBuilder) {
        addCommands();
    }

}
