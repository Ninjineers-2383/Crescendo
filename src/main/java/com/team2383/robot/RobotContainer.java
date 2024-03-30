// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.team2383.lib.util.mechanical_advantage.Alert;
import com.team2383.lib.util.mechanical_advantage.Alert.AlertType;
import com.team2383.robot.Constants.*;
import com.team2383.robot.commands.amp.ScoreAmpCommand;
import com.team2383.robot.commands.feeding.FullFeedCommand;
import com.team2383.robot.commands.feeding.IndexerBackOut;
import com.team2383.robot.commands.feeding.PartialFeedCommand;
import com.team2383.robot.commands.speaker.SeekAndShootCommand;
import com.team2383.robot.commands.speaker.ShootCommand;
import com.team2383.robot.commands.subsystem.drivetrain.*;
import com.team2383.robot.commands.subsystem.drivetrain.sysid.*;
import com.team2383.robot.commands.subsystem.feeder.*;
import com.team2383.robot.commands.subsystem.indexer.*;
import com.team2383.robot.commands.subsystem.orchestra.*;
import com.team2383.robot.commands.subsystem.piece_detection.DriveToPieceAuto;
import com.team2383.robot.commands.subsystem.piece_detection.DriveToPieceCommand;
import com.team2383.robot.commands.subsystem.pivot.*;
import com.team2383.robot.commands.subsystem.pivot.tuning.PivotSysIDCommand;
import com.team2383.robot.commands.subsystem.resting_hooks.RestingHooksPowerCommand;
import com.team2383.robot.commands.subsystem.shooter.*;
import com.team2383.robot.subsystems.cameraSim.*;
import com.team2383.robot.subsystems.drivetrain.*;
import com.team2383.robot.subsystems.drivetrain.SLAM.*;
import com.team2383.robot.subsystems.feeder.*;
import com.team2383.robot.subsystems.gamePieceSim.GamePieceSim;
import com.team2383.robot.subsystems.indexer.*;
import com.team2383.robot.subsystems.piece_detection.PieceDetectionIO;
import com.team2383.robot.subsystems.piece_detection.PieceDetectionIOPhoton;
import com.team2383.robot.subsystems.piece_detection.PieceDetectionSubsystem;
import com.team2383.robot.subsystems.pivot.*;
import com.team2383.robot.subsystems.resting_hooks.RestingHookIO;
import com.team2383.robot.subsystems.resting_hooks.RestingHookIOTalonSRX;
import com.team2383.robot.subsystems.resting_hooks.RestingHookSubsystem;
import com.team2383.robot.subsystems.shooter.*;
import com.team2383.robot.subsystems.sim_components.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final GenericHID m_driverController = new GenericHID(0);
    private final GenericHID m_operatorController = new GenericHID(1);

    private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
    private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 1).",
            AlertType.WARNING);

    private final Alert driverWrongController = new Alert("Driver controler not identified as Xbox controller (port 0)",
            AlertType.WARNING);
    private final Alert operatorWrongController = new Alert(
            "Operator controller identified as Xbox controller, should be F310 (port 1)", AlertType.WARNING);

    private final POVButton m_setHeadingForward = new POVButton(m_driverController, 0);
    private final POVButton m_setHeadingLeft = new POVButton(m_driverController, 270);
    private final POVButton m_setHeadingBackward = new POVButton(m_driverController, 180);
    private final POVButton m_setHeadingRight = new POVButton(m_driverController, 90);

    private final JoystickButton m_seek = new JoystickButton(m_operatorController, 2);

    private final JoystickButton m_fullFeedRear = new JoystickButton(m_driverController, 6);
    private final JoystickButton m_partialFeedRear = new JoystickButton(m_driverController, 5);

    private final JoystickButton m_manualAmpLineup = new JoystickButton(m_operatorController, 5);

    private final JoystickButton m_manualAmpShoot = new JoystickButton(m_operatorController, 6);

    private final JoystickButton m_pivotZero = new JoystickButton(m_driverController, 2);

    private final JoystickButton m_shoot = new JoystickButton(m_operatorController, 4);

    private final JoystickButton m_subwoofer = new JoystickButton(m_operatorController, 8);
    private final JoystickButton m_subwooferPivot = new JoystickButton(m_operatorController, 7);
    // private final JoystickButton m_mythicalTrap = new
    // JoystickButton(m_operatorController, 6);

    private final JoystickButton m_autoFeed = new JoystickButton(m_driverController, 1);

    private final JoystickButton m_resetHeading = new JoystickButton(m_driverController, Constants.OI.ResetHeading);

    private final JoystickButton m_headingToAprilTag = new JoystickButton(m_driverController,
            Constants.OI.HeadingToAprilTag);

    private final JoystickButton m_autoAmp = new JoystickButton(m_driverController, 3);

    private Trigger m_indexerBeamBreak;
    private Trigger m_feederBeamBreak;

    private final POVButton m_hooksDown = new POVButton(m_operatorController, 270);
    private final POVButton m_hooksUp = new POVButton(m_operatorController, 90);

    private final JoystickButton m_zeroBack = new JoystickButton(m_operatorController, 3);

    private DrivetrainSubsystem m_drivetrainSubsystem;

    private PivotSubsystem m_pivotSubsystem;

    private FeederSubsystem m_backFeederSubsystem;

    private IndexerSubsystem m_indexerSubsystem;

    private ShooterSubsystem m_shooterSubsystem;

    private PieceDetectionSubsystem m_pieceDetectionSubsystem;

    private RestingHookSubsystem m_restingHookSubsystem;

    private GamePieceSim m_gamePieceSimSubsystem;

    LoggedDashboardChooser<Boolean> enableLW = new LoggedDashboardChooser<Boolean>("Enable LW");

    LoggedDashboardNumber shooterTopBottomRPM = new LoggedDashboardNumber("Top Bottom RPM", 0);
    LoggedDashboardNumber shooterSideRPM = new LoggedDashboardNumber("Side RPM", 0);
    LoggedDashboardNumber shooterDifferentialRPM = new LoggedDashboardNumber("Differential RPM", 0);
    LoggedDashboardNumber pivotAngle = new LoggedDashboardNumber("Pivot Angle", 0);
    LoggedDashboardNumber trapArmAngle = new LoggedDashboardNumber("Trap Arm Angle", 0);

    LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<Command>("Auto Command");
    LoggedDashboardChooser<Command> testDashboardChooser = new LoggedDashboardChooser<Command>("Test Command");

    private boolean lwEnabled = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ROBOT_COMP:
                case ROBOT_PROTO:
                    m_drivetrainSubsystem = new DrivetrainSubsystem(
                            new GyroIONavX(),
                            new SwerveModuleIOFalcon500(DriveConstants.frontLeftConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.frontRightConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearLeftConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearRightConstants,
                                    Constants.kCANivoreBus));

                    m_pivotSubsystem = new PivotSubsystem(new PivotIOFalcon());

                    m_backFeederSubsystem = new FeederSubsystem(new FeederIONEO(FeederConstants.kRearMotorID));

                    m_indexerSubsystem = new IndexerSubsystem(new IndexerIONEO());

                    m_shooterSubsystem = new ShooterSubsystem(new ShooterIOFalcon500Neo());

                    m_pieceDetectionSubsystem = new PieceDetectionSubsystem(new PieceDetectionIOPhoton());

                    m_restingHookSubsystem = new RestingHookSubsystem(new RestingHookIOTalonSRX());

                    m_indexerBeamBreak = new Trigger(m_indexerSubsystem::isBeamBreakTripped);
                    m_feederBeamBreak = new Trigger(m_backFeederSubsystem::isBeamBreakTripped);

                    break;
                case ROBOT_SIM:
                    m_drivetrainSubsystem = new DrivetrainSubsystem(
                            new GyroIO() {},
                            new SwerveModuleIOSim(), new SwerveModuleIOSim(),
                            new SwerveModuleIOSim(), new SwerveModuleIOSim());
                    new CameraSimSubsystem("northstar-1", SLAMConstantsConfig.camTransforms[0],
                            m_drivetrainSubsystem::getDeadReckoningPose3d);
                    new CameraSimSubsystem("northstar-2", SLAMConstantsConfig.camTransforms[1],
                            m_drivetrainSubsystem::getDeadReckoningPose3d);
                    new CameraSimSubsystem("northstar-3", SLAMConstantsConfig.camTransforms[2],
                            m_drivetrainSubsystem::getDeadReckoningPose3d);
                    new CameraSimSubsystem("northstar-4", SLAMConstantsConfig.camTransforms[3],
                            m_drivetrainSubsystem::getDeadReckoningPose3d);

                    m_pivotSubsystem = new PivotSubsystem(new PivotIOSim());

                    m_backFeederSubsystem = new FeederSubsystem(new FeederIOSim());

                    m_indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());

                    m_shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());

                    m_gamePieceSimSubsystem = new GamePieceSim(m_drivetrainSubsystem::getEstimatorPose3d,
                            m_drivetrainSubsystem::getRobotRelativeSpeeds,
                            m_pivotSubsystem::getAngle,
                            m_shooterSubsystem::getTopBottomRPM,
                            m_fullFeedRear, m_partialFeedRear, m_indexerSubsystem::getPower);

                    m_indexerBeamBreak = new Trigger(m_gamePieceSimSubsystem::getIndexerBeamBreakTripped);
                    m_feederBeamBreak = new Trigger(m_gamePieceSimSubsystem::getFeederBeamBreakTripped);
                    break;
                default:
                    break;
            }
        }

        m_drivetrainSubsystem = m_drivetrainSubsystem == null
                ? new DrivetrainSubsystem(new GyroIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {},
                        new SwerveModuleIO() {}, new SwerveModuleIO() {})
                : m_drivetrainSubsystem;

        m_pivotSubsystem = m_pivotSubsystem == null ? new PivotSubsystem(new PivotIO() {}) : m_pivotSubsystem;

        m_backFeederSubsystem = m_backFeederSubsystem == null ? new FeederSubsystem(new FeederIO() {})
                : m_backFeederSubsystem;

        m_indexerSubsystem = m_indexerSubsystem == null ? new IndexerSubsystem(new IndexerIO() {}) : m_indexerSubsystem;

        m_shooterSubsystem = m_shooterSubsystem == null ? new ShooterSubsystem(new ShooterIO() {}) : m_shooterSubsystem;

        m_pieceDetectionSubsystem = m_pieceDetectionSubsystem == null
                ? new PieceDetectionSubsystem(new PieceDetectionIO() {})
                : m_pieceDetectionSubsystem;

        m_restingHookSubsystem = m_restingHookSubsystem == null
                ? new RestingHookSubsystem(new RestingHookIO() {})
                : m_restingHookSubsystem;

        m_gamePieceSimSubsystem = m_gamePieceSimSubsystem == null
                ? new GamePieceSim(m_drivetrainSubsystem::getEstimatorPose3d,
                        m_drivetrainSubsystem::getRobotRelativeSpeeds, m_pivotSubsystem::getAngle,
                        m_shooterSubsystem::getTopBottomRPM, m_fullFeedRear, m_partialFeedRear,
                        m_indexerSubsystem::getPower)
                : m_gamePieceSimSubsystem;

        m_indexerBeamBreak = m_indexerBeamBreak == null ? new Trigger(m_indexerSubsystem::isBeamBreakTripped)
                : m_indexerBeamBreak;

        m_feederBeamBreak = m_feederBeamBreak == null ? new Trigger(m_backFeederSubsystem::isBeamBreakTripped)
                : m_feederBeamBreak;

        new SimComponents(m_pivotSubsystem);

        configureDefaultCommands();

        registerAutoNamedCommands();

        registerAutoCommands();

        registerTestCommands();

        // Configure the button bindings
        configureButtonBindings();

        enableLW.addDefaultOption("No", false);
        enableLW.addOption("Yes", true);

    }

    public void periodic() {
        checkControllers();

        if (enableLW.get() && !lwEnabled) {
            LiveWindow.enableAllTelemetry();
            lwEnabled = true;
        } else if (!enableLW.get() && lwEnabled) {
            lwEnabled = false;
            LiveWindow.disableAllTelemetry();
        }
    }

    public void checkControllers() {
        // Check if theyre connected and if driver is xbox and if operator is NOT xbox
        driverDisconnected.set(
                !DriverStation.isJoystickConnected(m_driverController.getPort()));

        operatorDisconnected.set(
                !DriverStation.isJoystickConnected(m_operatorController.getPort()));

        driverWrongController.set(DriverStation.isJoystickConnected(m_driverController.getPort())
                && !DriverStation.getJoystickIsXbox(m_driverController.getPort()));

        operatorWrongController.set(DriverStation.isJoystickConnected(m_operatorController.getPort())
                && !DriverStation.getJoystickIsXbox(m_operatorController.getPort()));
    }

    private void configureButtonBindings() {
        m_setHeadingForward.whileTrue(new DrivetrainHeadingCommand(m_drivetrainSubsystem, new Rotation2d()));
        m_setHeadingBackward.whileTrue(new DrivetrainHeadingCommand(m_drivetrainSubsystem, new Rotation2d(Math.PI)));
        m_setHeadingLeft.whileTrue(new DrivetrainHeadingCommand(m_drivetrainSubsystem, new Rotation2d(Math.PI / 2)));
        m_setHeadingRight.whileTrue(new DrivetrainHeadingCommand(m_drivetrainSubsystem, new Rotation2d(-Math.PI / 2)));

        m_seek.and(m_feederBeamBreak).and(m_indexerBeamBreak.negate()).onTrue(
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(m_indexerBeamBreak),
                                        new WaitUntilCommand(m_indexerBeamBreak.negate())),
                                new FullFeedCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem,
                                        m_backFeederSubsystem, PivotPresets.FEED_BACK)),
                        new ParallelDeadlineGroup(
                                new IndexerBackOut(m_indexerSubsystem).withTimeout(0.4),
                                new ShooterRPMCommand(m_shooterSubsystem, () -> 0, () -> -800, () -> 0)),
                        new SeekAndShootCommand(m_drivetrainSubsystem, m_pivotSubsystem, m_shooterSubsystem,
                                m_indexerSubsystem, false),
                        new PivotZeroCommand(m_pivotSubsystem)));

        m_seek.and(m_indexerBeamBreak).toggleOnTrue(
                new SequentialCommandGroup(
                        new SeekAndShootCommand(m_drivetrainSubsystem, m_pivotSubsystem,
                                m_shooterSubsystem, m_indexerSubsystem, false),
                        new PivotZeroCommand(m_pivotSubsystem)));

        m_pivotZero.onTrue(new PivotZeroCommand(m_pivotSubsystem));

        new JoystickButton(m_operatorController, 1).onTrue(new PivotZeroCommand(m_pivotSubsystem));

        m_fullFeedRear.whileTrue(
                new FullFeedCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem,
                        m_backFeederSubsystem, PivotPresets.FEED_BACK));

        m_fullFeedRear.and(m_autoFeed).whileTrue(
                new DriveToPieceCommand(m_pieceDetectionSubsystem, m_drivetrainSubsystem, m_backFeederSubsystem));

        m_fullFeedRear.onFalse(new IndexerBackOut(m_indexerSubsystem).withTimeout(0.4)
                .deadlineWith(new ShooterRPMCommand(m_shooterSubsystem, () -> 0, () -> -800, () -> 0, true)
                        .andThen(new PivotPositionCommand(m_pivotSubsystem, PivotPresets.ZERO))));

        m_partialFeedRear.whileTrue(new PartialFeedCommand(m_backFeederSubsystem));

        m_partialFeedRear.and(m_autoFeed).whileTrue(
                new DriveToPieceCommand(m_pieceDetectionSubsystem, m_drivetrainSubsystem, m_backFeederSubsystem));

        m_partialFeedRear.onFalse(new FeederPowerCommand(m_backFeederSubsystem, () -> 0.2).withTimeout(0.2));

        m_shoot.onTrue(new ShootCommand(m_indexerSubsystem, m_shooterSubsystem).withTimeout(0.5));

        m_resetHeading.onTrue(new InstantCommand(() -> m_drivetrainSubsystem.resetHeading()));

        m_headingToAprilTag.onTrue(
                new InstantCommand(
                        () -> m_drivetrainSubsystem
                                .forceHeading(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                                        ? m_drivetrainSubsystem.getPose().getRotation()
                                        : m_drivetrainSubsystem.getPose().getRotation()
                                                .plus(new Rotation2d(Math.PI)))));

        m_autoAmp.whileTrue(
                new ScoreAmpCommand(m_drivetrainSubsystem, m_pivotSubsystem, m_shooterSubsystem, m_indexerSubsystem));

        m_manualAmpLineup.whileTrue(
                new ParallelCommandGroup(
                        new PivotPositionCommand(m_pivotSubsystem,
                                PivotPresets.SCORE_AMP)));

        m_manualAmpShoot.whileTrue(
                // new ParallelDeadlineGroup(
                // new SequentialCommandGroup(
                // new WaitCommand(0.5),
                new ShooterRPMCommand(m_shooterSubsystem, () -> -700, () -> 750, () -> 250)
                        .alongWith(new IndexerCommand(m_indexerSubsystem, () -> -0.5)));

        m_manualAmpShoot.onFalse(new PivotPositionCommand(m_pivotSubsystem, PivotPresets.ZERO));

        // .andThen(new SequentialCommandGroup(
        // // new PivotPositionCommand(m_pivotSubsystem,
        // // PivotPresets.ZERO).withTimeout(0.5),
        // new PivotPositionCommand(m_pivotSubsystem,
        // Math.toRadians(90)).withTimeout(0.5),
        // new PivotPositionCommand(m_pivotSubsystem,
        // PivotPresets.ZERO).withTimeout(0.25)))

        m_hooksDown.whileTrue(
                new ParallelCommandGroup(
                        new PivotPositionCommand(m_pivotSubsystem, PivotPresets.ZERO_BACK),
                        new RestingHooksPowerCommand(m_restingHookSubsystem,
                                () -> Math.abs(m_operatorController.getRawAxis(0)),
                                () -> MathUtil.applyDeadband(Math.abs(m_operatorController.getRawAxis(4)), 0.15))));

        m_hooksUp.whileTrue(
                new ParallelCommandGroup(
                        new PivotPositionCommand(m_pivotSubsystem, PivotPresets.ZERO_BACK),
                        new RestingHooksPowerCommand(m_restingHookSubsystem,
                                () -> -Math.abs(m_operatorController.getRawAxis(0)),
                                () -> -MathUtil.applyDeadband(Math.abs(m_operatorController.getRawAxis(4)), 0.15))));

        m_zeroBack.onTrue(new PivotPositionCommand(m_pivotSubsystem, PivotPresets.ZERO_BACK));

        new JoystickButton(m_operatorController, 9)
                .whileTrue(new PivotVelocityCommand(m_pivotSubsystem, () -> m_operatorController.getRawAxis(0)));

        m_indexerBeamBreak.or(m_feederBeamBreak).onTrue(
                new SequentialCommandGroup(
                        new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 1)),
                        new WaitCommand(0.25),
                        new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0))));

        // Command subwooferPivot = new ConditionalCommand(
        // new PivotPositionCommand(m_pivotSubsystem,
        // PivotPresets.SUBWOOFER_BACK),
        // new PivotPositionCommand(m_pivotSubsystem,
        // PivotPresets.SUBWOOFER),
        // () -> MathUtil.angleModulus(m_drivetrainSubsystem.getHeading().getRadians())
        // < (Math.PI / 2.0) &&
        // MathUtil.angleModulus(m_drivetrainSubsystem
        // .getHeading()
        // .getRadians()) > -(Math.PI / 2.0));
        m_subwoofer.onTrue(
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new PivotPositionCommand(m_pivotSubsystem,
                                        PivotPresets.SUBWOOFER),
                                new PivotPositionCommand(m_pivotSubsystem,
                                        PivotPresets.SUBWOOFER_BACK),
                                () -> MathUtil
                                        .angleModulus(m_drivetrainSubsystem.getHeading().getRadians()) < (Math.PI / 2.0)
                                        &&
                                        MathUtil.angleModulus(m_drivetrainSubsystem
                                                .getHeading()
                                                .getRadians()) > -(Math.PI
                                                        / 2.0)),
                        // new PivotPositionCommand(m_pivotSubsystem, PivotPresets.SUBWOOFER_BACK),
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(0.04),
                                        new WaitUntilCommand(() -> m_shooterSubsystem.isFinished()
                                                && m_pivotSubsystem.isFinished())),
                                new ShooterRPMCommand(m_shooterSubsystem, () -> -4000, () -> 2000, () -> 0)),
                        new IndexerCommand(m_indexerSubsystem, () -> -1.0).withTimeout(0.4),
                        new ShooterRPMCommand(m_shooterSubsystem, () -> 0, () -> 0, () -> 0).withTimeout(0.02),
                        new PivotPositionCommand(m_pivotSubsystem, PivotPresets.ZERO)));

        m_subwooferPivot.onTrue(new ConditionalCommand(
                new PivotPositionCommand(m_pivotSubsystem,
                        PivotPresets.SUBWOOFER),
                new PivotPositionCommand(m_pivotSubsystem,
                        PivotPresets.SUBWOOFER_BACK),
                () -> MathUtil.angleModulus(m_drivetrainSubsystem.getHeading().getRadians()) < (Math.PI / 2.0) &&
                        MathUtil.angleModulus(m_drivetrainSubsystem
                                .getHeading()
                                .getRadians()) > -(Math.PI / 2.0)).alongWith(
                                        new ShooterRPMCommand(m_shooterSubsystem, () -> -4000, () -> 2000, () -> 0)));
        // m_mythicalTrap.onTrue(
        // new SequentialCommandGroup(
        // new PivotPositionCommand(m_pivotSubsystem, PivotPresets.SCORE_TRAP),
        // new ParallelDeadlineGroup(
        // new SequentialCommandGroup(
        // new WaitCommand(0.04),
        // new WaitUntilCommand(() -> m_shooterSubsystem.isFinished()
        // && m_pivotSubsystem.isFinished())),
        // new ShooterRPMCommand(m_shooterSubsystem, () -> -2000, () -> 500, () -> 0)),
        // new IndexerCommand(m_indexerSubsystem, () -> -1.0).withTimeout(0.4),
        // new ShooterRPMCommand(m_shooterSubsystem, () -> 0, () -> 0, () ->
        // 0).withTimeout(0.02)
        // // new PivotPositionCommand(m_pivotSubsystem, PivotPresets.ZERO_BACK)
        // ));

    }

    private void configureDefaultCommands() {
        m_drivetrainSubsystem.setDefaultCommand(
                new JoystickDriveCommand(m_drivetrainSubsystem,
                        () -> new ChassisSpeeds(
                                MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveX),
                                        .1),
                                MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveY),
                                        .1),
                                Math.toRadians(100 * MathUtil
                                        .applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveOmega), 0.1))),
                        () -> true));

        m_pivotSubsystem.setDefaultCommand(
                new PivotDefaultCommand(m_pivotSubsystem,
                        () -> pivotAngle.get() * (Math.PI / 180)));

        m_backFeederSubsystem.setDefaultCommand(new FeederPowerCommand(m_backFeederSubsystem,
                () -> 0));

        m_indexerSubsystem
                .setDefaultCommand(
                        new IndexerCommand(m_indexerSubsystem,
                                () -> MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.1)));

        m_shooterSubsystem.setDefaultCommand(
                new ShooterRPMCommand(m_shooterSubsystem, shooterTopBottomRPM::get, shooterSideRPM::get,
                        shooterDifferentialRPM::get));
    }

    public void disable() {
        m_pivotSubsystem.disable();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public Command getTestCommand() {
        return testDashboardChooser.get();
    }

    private void registerAutoCommands() {
        autoChooser.addDefaultOption("None", (Command) null);

        autoChooser.addOption("3.5Amp", new PathPlannerAuto("3.5Amp"));

        autoChooser.addOption("3Amp", new PathPlannerAuto("3Amp"));

        autoChooser.addOption("3SourceTapeBottomTop", new PathPlannerAuto("3SourceTapeBottomTop"));

        autoChooser.addOption("3SourceTapeTopBottom", new PathPlannerAuto("3SourceTapeTopBottom"));

        autoChooser.addOption("3SourceSWBottomTop", new PathPlannerAuto("3SourceSWBottomTop"));

        autoChooser.addOption("3SourceSWTopBottom", new PathPlannerAuto("3SourceSWTopBottom"));

        autoChooser.addOption("TwoPieceCenter", new PathPlannerAuto("TwoPieceCenter"));

        autoChooser.addOption("ThreePieceCenter", new PathPlannerAuto("ThreePieceCenter"));

        autoChooser.addOption("FourPieceCenter", new PathPlannerAuto("FourPieceCenter"));

        autoChooser.addOption("4.5PieceCenter", new PathPlannerAuto("4.5PieceCenter"));

    }

    private void registerTestCommands() {
        DrivetrainSysIDCommand drivetrainSysIDCommand = new DrivetrainSysIDCommand(m_drivetrainSubsystem);

        testDashboardChooser.addDefaultOption("None", (Command) null);

        testDashboardChooser.addOption("Drivetrain Dynamic Forward",
                drivetrainSysIDCommand.getDynamic(Direction.kForward));

        testDashboardChooser.addOption("Drivetrain Dynamic Reverse",
                drivetrainSysIDCommand.getDynamic(Direction.kReverse));

        testDashboardChooser.addOption("Drivetrain Quasistatic Forward",
                drivetrainSysIDCommand.getQuasiStatic(Direction.kForward));

        testDashboardChooser.addOption("Drivetrain Quasistatic Reverse",
                drivetrainSysIDCommand.getQuasiStatic(Direction.kReverse));

        ShooterSysIDCommand shooterSysIDCommand = new ShooterSysIDCommand(m_shooterSubsystem);

        testDashboardChooser.addOption("Shooter Dynamic Forward",
                shooterSysIDCommand.getDynamic(Direction.kForward));

        testDashboardChooser.addOption("Shooter Dynamic Reverse",
                shooterSysIDCommand.getDynamic(Direction.kReverse));

        testDashboardChooser.addOption("Shooter Quasistatic Forward",
                shooterSysIDCommand.getQuasiStatic(Direction.kForward));

        testDashboardChooser.addOption("Shooter Quasistatic Reverse",
                shooterSysIDCommand.getQuasiStatic(Direction.kReverse));

        testDashboardChooser.addOption("Sea Shanty 2", new OrchestraCommand("music/SeaShanty2.chrp",
                m_drivetrainSubsystem, m_pivotSubsystem, m_backFeederSubsystem, m_indexerSubsystem,
                m_shooterSubsystem));

        testDashboardChooser.addOption("Drivetrain Heading Tuning",
                new DrivetrainHeadingControllerCommand(m_drivetrainSubsystem, () -> Math.toRadians(100 * MathUtil
                        .applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveOmega), 0.1)),
                        () -> m_driverController.getRawButton(9)));

        PivotSysIDCommand pivotSysID = new PivotSysIDCommand(m_pivotSubsystem);

        testDashboardChooser.addOption("Pivot Quasi Forward", pivotSysID.getQuasiStatic(Direction.kForward));
        testDashboardChooser.addOption("Pivot Quasi Backward", pivotSysID.getQuasiStatic(Direction.kReverse));
        testDashboardChooser.addOption("Pivot Dyn Forward", pivotSysID.getDynamic(Direction.kForward));
        testDashboardChooser.addOption("Pivot Dyn Backward", pivotSysID.getDynamic(Direction.kReverse));

        testDashboardChooser.addOption("Wheel Radius Characterization Clockwise",
                new WheelRadiusCharacterization(m_drivetrainSubsystem, m_drivetrainSubsystem::getHeading,
                        WheelRadiusCharacterization.Direction.CLOCKWISE));

        testDashboardChooser.addOption("Wheel Radius Characterization Counter Clockwise",
                new WheelRadiusCharacterization(m_drivetrainSubsystem, m_drivetrainSubsystem::getHeading,
                        WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE));
    }

    public void registerAutoNamedCommands() {
        NamedCommands.registerCommand("InitializeHeading",
                new RunCommand(
                        () -> m_drivetrainSubsystem
                                .forceHeading(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                                        ? m_drivetrainSubsystem.getPose().getRotation()
                                        : m_drivetrainSubsystem.getPose().getRotation()
                                                .plus(new Rotation2d(Math.PI))),
                        m_drivetrainSubsystem).withTimeout(0.02));

        NamedCommands.registerCommand("SeekAndShoot",
                new SequentialCommandGroup(
                        new RunCommand(
                                () -> m_drivetrainSubsystem
                                        .forceHeading(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                                                ? m_drivetrainSubsystem.getPose().getRotation()
                                                : m_drivetrainSubsystem.getPose().getRotation()
                                                        .plus(new Rotation2d(Math.PI))),
                                m_drivetrainSubsystem).withTimeout(0.02),
                        new SeekAndShootCommand(m_drivetrainSubsystem, m_pivotSubsystem, m_shooterSubsystem,
                                m_indexerSubsystem, true),
                        new PivotPositionCommand(m_pivotSubsystem, PivotPresets.ZERO)));

        NamedCommands.registerCommand("DriveToPiece",
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new DriveToPieceAuto(m_pieceDetectionSubsystem, m_drivetrainSubsystem,
                                        m_backFeederSubsystem),
                                new FullFeedCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem,
                                        m_backFeederSubsystem, PivotPresets.FEED_BACK)),
                        new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(), false, false),
                                m_drivetrainSubsystem),
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(m_indexerBeamBreak),
                                        new WaitUntilCommand(m_indexerBeamBreak.negate())),
                                new FullFeedCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem,
                                        m_backFeederSubsystem, PivotPresets.FEED_BACK)),
                        new IndexerBackOut(m_indexerSubsystem),
                        new PivotPositionCommand(m_pivotSubsystem, PivotPresets.ZERO)));

        NamedCommands.registerCommand("StealShoot",
                new SequentialCommandGroup(
                        new PivotPositionCommand(m_pivotSubsystem, PivotPresets.STEAL_MID),
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(0.04),
                                        new WaitUntilCommand(() -> m_shooterSubsystem.isFinished()
                                                && m_pivotSubsystem.isFinished())),
                                new ShooterRPMCommand(m_shooterSubsystem, () -> -3500, () -> 2000, () -> 0)),
                        new IndexerCommand(m_indexerSubsystem, () -> -1.0).withTimeout(0.4),
                        new ShooterRPMCommand(m_shooterSubsystem, () -> 0, () -> 0, () -> 0).withTimeout(0.02),
                        new PivotPositionCommand(m_pivotSubsystem, PivotPresets.ZERO_BACK)));

        NamedCommands.registerCommand("ShootSubwooferStart", new SequentialCommandGroup(
                new PivotPositionCommand(m_pivotSubsystem, PivotPresets.SUBWOOFER_BACK),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(0.04),
                                new WaitUntilCommand(() -> m_shooterSubsystem.isFinished()
                                        && m_pivotSubsystem.isFinished())).withTimeout(1.0),
                        new ShooterRPMCommand(m_shooterSubsystem, () -> -4000, () -> 2000, () -> 0)),
                new IndexerCommand(m_indexerSubsystem, () -> -1.0).withTimeout(0.4),
                new ShooterRPMCommand(m_shooterSubsystem, () -> 0, () -> 0, () -> 0).withTimeout(0.02)));

        NamedCommands.registerCommand("ShootSubwoofer", new SequentialCommandGroup(
                new PivotPositionCommand(m_pivotSubsystem, PivotPresets.SUBWOOFER_BACK),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(0.04),
                                new WaitUntilCommand(() -> m_shooterSubsystem.isFinished()
                                        && m_pivotSubsystem.isFinished())).withTimeout(0.5),
                        new ShooterRPMCommand(m_shooterSubsystem, () -> -4000, () -> 2000, () -> 0)),
                new IndexerCommand(m_indexerSubsystem, () -> -1.0).withTimeout(0.4),
                new ShooterRPMCommand(m_shooterSubsystem, () -> 0, () -> 0, () -> 0).withTimeout(0.02)));

        NamedCommands.registerCommand("FullFeed",
                new FullFeedCommand(m_shooterSubsystem, m_indexerSubsystem,
                        m_pivotSubsystem, m_backFeederSubsystem, PivotPresets.SUBWOOFER_BACK)
                                .until(m_indexerBeamBreak).andThen(new IndexerBackOut(m_indexerSubsystem)
                                        .alongWith(new PivotPositionCommand(m_pivotSubsystem,
                                                PivotPresets.SUBWOOFER_BACK))));
    }
}
