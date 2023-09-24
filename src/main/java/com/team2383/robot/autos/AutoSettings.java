package com.team2383.robot.autos;
// package com.team2383.nunchuck.autos;

// import java.util.HashMap;

// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import com.team2383.nunchuck.commands.FeederCommand;
// import com.team2383.nunchuck.commands.pinkArm.PinkArmPresetCommand;
// import com.team2383.nunchuck.commands.pinkArm.position.PositionConstants;
// import com.team2383.nunchuck.subsystems.drivetrain.DrivetrainSubsystem;
// import com.team2383.nunchuck.subsystems.pinkArm.feeder.FeederSubsystem;
// import com.team2383.nunchuck.subsystems.pinkArm.pivot.PivotSubsystem;
// import com.team2383.nunchuck.subsystems.pinkArm.telescope.TelescopeSubsystem;
// import com.team2383.nunchuck.subsystems.pinkArm.wrist.WristSubsystem;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class AutoSettings {

// private static PivotSubsystem m_pivotSubsystem;
// private static TelescopeSubsystem m_telescopeSubsystem;
// private static WristSubsystem m_wristSubsystem;
// private static FeederSubsystem m_feederSubsystem;

// private static DrivetrainSubsystem m_drivetrainSubsystem;

// // public static final HashMap<String, Command> autoHashMap = new HashMap<>()
// {
// // {
// // put("Auto Log", new PrintCommand("Auto Event: log"));

// // put("Feed Cone",
// // new SequentialCommandGroup(
// // new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem,
// m_wristSubsystem,
// // PositionConstants.FEED_UPRIGHT_CONE),
// // new FeederCommand(m_feederSubsystem, () -> 1).withTimeout(0.7)));
// // put("Feed Cube",
// // new SequentialCommandGroup(
// // new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem,
// m_wristSubsystem,
// // PositionConstants.FEED_CONE_POS),
// // new FeederCommand(m_feederSubsystem, () -> 1).withTimeout(0.7)));

// // put("Score Cone Low",
// // new SequentialCommandGroup(
// // new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem,
// m_wristSubsystem,
// // PositionConstants.LOW_SCORE_POS),
// // new FeederCommand(m_feederSubsystem, () -> -1).withTimeout(0.7)));
// // put("Score Cone Mid",
// // new SequentialCommandGroup(
// // new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem,
// m_wristSubsystem,
// // PositionConstants.MID_SCORE_POS),
// // new FeederCommand(m_feederSubsystem, () -> -1).withTimeout(0.7)));
// // put("Score Cone High",
// // new SequentialCommandGroup(
// // new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem,
// m_wristSubsystem,
// // PositionConstants.HIGH_SCORE_POS),
// // new FeederCommand(m_feederSubsystem, () -> -1).withTimeout(0.7)));

// // put("Score Cube Low",
// // new SequentialCommandGroup(
// // new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem,
// m_wristSubsystem,
// // PositionConstants.LOW_SCORE_POS),
// // new FeederCommand(m_feederSubsystem, () -> -0.6).withTimeout(0.7)));
// // put("Score Cube Mid",
// // new SequentialCommandGroup(
// // new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem,
// m_wristSubsystem,
// // PositionConstants.MID_SCORE_POS),
// // new FeederCommand(m_feederSubsystem, () -> -0.6).withTimeout(0.7)));
// // put("Score Cube High",
// // new SequentialCommandGroup(
// // new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem,
// m_wristSubsystem,
// // PositionConstants.HIGH_SCORE_POS),
// // new FeederCommand(m_feederSubsystem, () -> -0.6).withTimeout(0.7)));
// // }
// // };

// // public static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
// // m_drivetrainSubsystem::getPose,
// // m_drivetrainSubsystem::forceOdometry,
// // m_drivetrainSubsystem.m_kinematics,
// // new PIDConstants(5, 0, 0),
// // new PIDConstants(0.5, 0, 0),
// // m_drivetrainSubsystem::setModuleStates,
// // AutoSettings.autoHashMap,
// // true,
// // m_drivetrainSubsystem);

// }
