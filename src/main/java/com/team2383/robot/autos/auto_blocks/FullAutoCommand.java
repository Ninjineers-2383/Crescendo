package com.team2383.robot.autos.auto_blocks;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.robot.commands.FeederVoltageCommand;
import com.team2383.robot.commands.blizzard.BlizzardCommand;
import com.team2383.robot.commands.blizzard.BlizzardPresets;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullAutoCommand extends SequentialCommandGroup {
    public FullAutoCommand(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, WristSubsystem wrist,
            FeederSubsystem feeder, String pathName) {

        HashMap<String, Command> autoHashMap = new HashMap<>() {
            {
                put("Auto Log", new PrintCommand("Auto Event: log"));

                put("Feed Cone", new BlizzardCommand(elevator, wrist, BlizzardPresets.GROUND_INTAKE)
                        .alongWith(new FeederVoltageCommand(feeder, () -> -0.5, false)));

                put("Feed Cube", new BlizzardCommand(elevator, wrist, BlizzardPresets.GROUND_INTAKE)
                        .alongWith(new FeederVoltageCommand(feeder, () -> -0.5, true)));

                put("Retract", new BlizzardCommand(elevator, wrist, BlizzardPresets.CONE_CHUTE));

                put("Score Cube High", new ScoreHighCommand(elevator, wrist, feeder, true));

                put("Score Cone High", new ScoreHighCommand(elevator, wrist, feeder, false));
            }
        };

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                drivetrain::getPose,
                drivetrain::forceOdometry,
                drivetrain.m_kinematics,
                new PIDConstants(5, 0, 0),
                new PIDConstants(4, 0, 0),
                drivetrain::setModuleStates,
                autoHashMap,
                true,
                drivetrain);

        // This will load the file "FullAuto.path" and generate it with a max velocity
        // and a max acceleration
        // for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName,
                new PathConstraints(6, 1));

        addCommands(
                autoBuilder.fullAuto(pathGroup));
    }
}
