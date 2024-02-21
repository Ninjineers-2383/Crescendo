// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.team2383.robot.Constants.*;
import com.team2383.robot.commands.amp.ScoreAmpCommand;
import com.team2383.robot.commands.feeding.FullFeedBackCommand;
import com.team2383.robot.commands.feeding.FullFeedFrontCommand;
import com.team2383.robot.commands.speaker.SeekAndShootCommand;
import com.team2383.robot.commands.speaker.SeekCommand;
import com.team2383.robot.commands.speaker.ShootCommand;
import com.team2383.robot.commands.subsystem.drivetrain.*;
import com.team2383.robot.commands.subsystem.drivetrain.sysid.*;
import com.team2383.robot.commands.subsystem.feeder.*;
import com.team2383.robot.commands.subsystem.indexer.*;
import com.team2383.robot.commands.subsystem.orchestra.*;
import com.team2383.robot.commands.subsystem.pivot.*;
import com.team2383.robot.commands.subsystem.pivot.tuning.PivotTuningCommand;
import com.team2383.robot.commands.subsystem.shooter.*;
import com.team2383.robot.commands.subsystem.trap_arm.TrapArmPositionCommand;
import com.team2383.robot.commands.subsystem.trap_arm.tuning.TrapArmTuningCommand;
import com.team2383.robot.subsystems.cameraSim.*;
import com.team2383.robot.subsystems.drivetrain.*;
import com.team2383.robot.subsystems.drivetrain.SLAM.*;
import com.team2383.robot.subsystems.feeder.*;
import com.team2383.robot.subsystems.gamePieceSim.GamePieceSimSubsystem;
import com.team2383.robot.subsystems.indexer.*;
import com.team2383.robot.subsystems.pivot.*;
import com.team2383.robot.subsystems.resting_hooks.RestingHookIO;
import com.team2383.robot.subsystems.resting_hooks.RestingHookIOTalonSRX;
import com.team2383.robot.subsystems.resting_hooks.RestingHookSubsystem;
import com.team2383.robot.subsystems.shooter.*;
import com.team2383.robot.subsystems.sim_components.*;
import com.team2383.robot.subsystems.trap_arm.TrapArmIO;
import com.team2383.robot.subsystems.trap_arm.TrapArmIOTalonSRXTrapezoidal;
import com.team2383.robot.subsystems.trap_arm.TrapArmSubsystem;
import com.team2383.robot.subsystems.trap_feeder.TrapFeederIO;
import com.team2383.robot.subsystems.trap_feeder.TrapFeederIONeo550;
import com.team2383.robot.subsystems.trap_feeder.TrapFeederSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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

    private final JoystickButton m_setHeadingZero = new JoystickButton(m_driverController, 1);

    private final JoystickButton m_seek = new JoystickButton(m_operatorController, 2);

    private final JoystickButton m_fullFeedFront = new JoystickButton(m_driverController, 5);
    private final JoystickButton m_fullFeedRear = new JoystickButton(m_driverController, 6);

    private final JoystickButton m_pivotZero = new JoystickButton(m_operatorController, 1);
    private final POVButton m_feedLeft = new POVButton(m_operatorController, 0);

    private final JoystickButton m_shoot = new JoystickButton(m_operatorController, 4);

    // private final JoystickButton

    private DrivetrainSubsystem m_drivetrainSubsystem;

    private PivotSubsystem m_pivotSubsystem;

    private FeederSubsystem m_frontFeederSubsystem;
    private FeederSubsystem m_backFeederSubsystem;

    private IndexerSubsystem m_indexerSubsystem;

    private ShooterSubsystem m_shooterSubsystem;

    private TrapArmSubsystem m_trapArmSubsystem;
    private TrapFeederSubsystem m_trapFeederSubsystem;

    private RestingHookSubsystem m_restingHookSubsystem;

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
                    m_drivetrainSubsystem = new DrivetrainSubsystem(
                            new GyroIOPigeon(0, Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.frontLeftConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.frontRightConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearLeftConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearRightConstants,
                                    Constants.kCANivoreBus));

                    m_pivotSubsystem = new PivotSubsystem(new PivotIOFalconTrapezoidal());

                    m_frontFeederSubsystem = new FeederSubsystem(new FeederIONEO(FeederConstants.kFrontMotorID));
                    m_backFeederSubsystem = new FeederSubsystem(new FeederIONEO(FeederConstants.kRearMotorID));

                    m_indexerSubsystem = new IndexerSubsystem(new IndexerIONEO());

                    m_shooterSubsystem = new ShooterSubsystem(new ShooterIOFalcon500Neo());

                    break;
                case ROBOT_PROTO:
                    m_drivetrainSubsystem = new DrivetrainSubsystem(
                            new GyroIOPigeon(0, Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.frontLeftConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.frontRightConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearLeftConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearRightConstants,
                                    Constants.kCANivoreBus));

                    m_pivotSubsystem = new PivotSubsystem(new PivotIOFalconTrapezoidal());

                    m_frontFeederSubsystem = new FeederSubsystem(new FeederIONEO(FeederConstants.kFrontMotorID));

                    m_indexerSubsystem = new IndexerSubsystem(new IndexerIONEO());

                    m_shooterSubsystem = new ShooterSubsystem(new ShooterIOFalcon500Neo());

                    m_trapArmSubsystem = new TrapArmSubsystem(new TrapArmIOTalonSRXTrapezoidal());

                    m_trapFeederSubsystem = new TrapFeederSubsystem(new TrapFeederIONeo550());

                    m_restingHookSubsystem = new RestingHookSubsystem(new RestingHookIOTalonSRX());

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

                    m_frontFeederSubsystem = new FeederSubsystem(new FeederIOSim());
                    m_backFeederSubsystem = new FeederSubsystem(new FeederIOSim());

                    m_indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());

                    m_shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());

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

        m_frontFeederSubsystem = m_frontFeederSubsystem == null ? new FeederSubsystem(new FeederIO() {})
                : m_frontFeederSubsystem;

        m_backFeederSubsystem = m_backFeederSubsystem == null ? new FeederSubsystem(new FeederIO() {})
                : m_backFeederSubsystem;

        m_indexerSubsystem = m_indexerSubsystem == null ? new IndexerSubsystem(new IndexerIO() {}) : m_indexerSubsystem;

        m_shooterSubsystem = m_shooterSubsystem == null ? new ShooterSubsystem(new ShooterIO() {}) : m_shooterSubsystem;

        m_trapArmSubsystem = m_trapArmSubsystem == null ? new TrapArmSubsystem(new TrapArmIO() {}) : m_trapArmSubsystem;

        m_trapFeederSubsystem = m_trapFeederSubsystem == null ? new TrapFeederSubsystem(new TrapFeederIO() {})
                : m_trapFeederSubsystem;

        m_restingHookSubsystem = m_restingHookSubsystem == null ? new RestingHookSubsystem(new RestingHookIO() {})
                : m_restingHookSubsystem;

        new SimComponents(m_pivotSubsystem);

        new GamePieceSimSubsystem(m_drivetrainSubsystem::getEstimatorPose3d,
                m_drivetrainSubsystem::getRobotRelativeSpeeds, m_shoot, () -> m_frontFeederSubsystem.getPower() > 0,
                () -> m_backFeederSubsystem.getPower() > 0,
                m_pivotSubsystem::getAngle,
                m_shooterSubsystem::getTopBottomRPM);

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
        // autoChooser.periodic();

        if (enableLW.get() && !lwEnabled) {
            LiveWindow.enableAllTelemetry();
            lwEnabled = true;
        } else if (!enableLW.get() && lwEnabled) {
            lwEnabled = false;
            LiveWindow.disableAllTelemetry();
        }
    }

    private void configureButtonBindings() {
        m_setHeadingZero.whileTrue(new DrivetrainHeadingCommand(m_drivetrainSubsystem, new Rotation2d()));

        m_seek.toggleOnTrue(new SeekCommand(m_drivetrainSubsystem, m_pivotSubsystem, m_shooterSubsystem, false));

        m_pivotZero.onTrue(new PivotPositionCommand(m_pivotSubsystem, PivotPresets.ZERO));
        m_feedLeft.onTrue(new PivotPositionCommand(m_pivotSubsystem, PivotPresets.FEED_FRONT));

        m_fullFeedFront.whileTrue(
                new FullFeedFrontCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem,
                        m_frontFeederSubsystem));

        m_fullFeedRear.whileTrue(
                new FullFeedBackCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem,
                        m_backFeederSubsystem));

        m_fullFeedFront.onFalse(new IndexerCommand(m_indexerSubsystem, () -> 0.2).withTimeout(0.1));

        m_fullFeedRear.onFalse(new IndexerCommand(m_indexerSubsystem, () -> 0.2).withTimeout(0.1));

        m_shoot.onTrue(new ShootCommand(m_indexerSubsystem).withTimeout(0.5));

        new JoystickButton(m_driverController, Constants.OI.ResetHeading)
                .onTrue(new InstantCommand(() -> m_drivetrainSubsystem.resetHeading()));

        new JoystickButton(m_driverController, Constants.OI.HeadingToAprilTag).onTrue(
                new InstantCommand(
                        () -> m_drivetrainSubsystem
                                .forceHeading(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                                        ? m_drivetrainSubsystem.getPose().getRotation()
                                        : m_drivetrainSubsystem.getPose().getRotation()
                                                .plus(new Rotation2d(Math.PI)))));

        new JoystickButton(m_driverController, 3).whileTrue(
                new ScoreAmpCommand(m_drivetrainSubsystem, m_pivotSubsystem, m_shooterSubsystem, m_indexerSubsystem));

        new JoystickButton(m_operatorController, 3).onTrue(new IndexerCommand(m_indexerSubsystem, () -> 0.7))
                .onFalse(new IndexerCommand(m_indexerSubsystem, () -> 0));

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
                        () -> !(m_driverController.getRawButton(Constants.OI.FieldCentric))));

        m_pivotSubsystem.setDefaultCommand(
                new PivotDefaultCommand(m_pivotSubsystem,
                        () -> pivotAngle.get() * (Math.PI / 180)));

        m_frontFeederSubsystem.setDefaultCommand(new FeederPowerCommand(m_frontFeederSubsystem,
                () -> 0));

        m_backFeederSubsystem.setDefaultCommand(new FeederPowerCommand(m_backFeederSubsystem,
                () -> 0));

        m_indexerSubsystem
                .setDefaultCommand(
                        new IndexerCommand(m_indexerSubsystem, () -> m_operatorController.getRawAxis(1)));

        m_shooterSubsystem.setDefaultCommand(
                new ShooterRPMCommand(m_shooterSubsystem, shooterTopBottomRPM::get, shooterSideRPM::get,
                        shooterDifferentialRPM::get));

        m_trapArmSubsystem.setDefaultCommand(
                new TrapArmPositionCommand(m_trapArmSubsystem, () -> trapArmAngle.get() * (Math.PI / 180)));
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

        autoChooser.addOption("Two Note", new PathPlannerAuto("TwoPieceTop"));

        autoChooser.addOption("Three Note", new PathPlannerAuto("ThreePieceTop"));
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

        testDashboardChooser.addOption("Shooter Dynamic Forward",
                m_shooterSubsystem.getDynamic(Direction.kForward));

        testDashboardChooser.addOption("Shooter Dynamic Reverse",
                m_shooterSubsystem.getDynamic(Direction.kReverse));

        testDashboardChooser.addOption("Shooter Quasistatic Forward",
                m_shooterSubsystem.getQuasiStatic(Direction.kForward));

        testDashboardChooser.addOption("Shooter Quasistatic Reverse",
                m_shooterSubsystem.getQuasiStatic(Direction.kReverse));

        testDashboardChooser.addOption("Pivot Tuning",
                new PivotTuningCommand(m_pivotSubsystem, () -> m_operatorController.getRawAxis(0),
                        () -> m_operatorController.getRawButton(1)));

        testDashboardChooser.addOption("Sea Shanty 2", new OrchestraCommand("music/SeaShanty2.chrp",
                m_drivetrainSubsystem, m_pivotSubsystem, m_frontFeederSubsystem, m_indexerSubsystem,
                m_shooterSubsystem));

        testDashboardChooser.addOption("Super Mario Bros Overworld Theme",
                new OrchestraCommand("music/MarioOverworld.chrp",
                        m_drivetrainSubsystem, m_pivotSubsystem, m_frontFeederSubsystem, m_indexerSubsystem,
                        m_shooterSubsystem));

        testDashboardChooser.addOption("Drivetrain Heading Tuning",
                new DrivetrainHeadingControllerCommand(m_drivetrainSubsystem, () -> Math.toRadians(100 * MathUtil
                        .applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveOmega), 0.1)),
                        () -> m_driverController.getRawButton(9)));

        testDashboardChooser.addOption("Trap Arm Tuning", new TrapArmTuningCommand(m_trapArmSubsystem));
    }

    public void registerAutoNamedCommands() {
        NamedCommands.registerCommand("FeedFront",
                new FullFeedFrontCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem,
                        m_frontFeederSubsystem));

        NamedCommands.registerCommand("FeedBack",
                new FullFeedBackCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem,
                        m_backFeederSubsystem));

        NamedCommands.registerCommand("SeekAndShoot",
                new SeekAndShootCommand(m_drivetrainSubsystem, m_pivotSubsystem, m_shooterSubsystem,
                        m_indexerSubsystem));

        NamedCommands.registerCommand("StartShooter",
                new ShooterRPMCommand(m_shooterSubsystem, () -> 6000, () -> 1000, () -> 0));

        NamedCommands.registerCommand("StopShooter",
                new ShooterRPMCommand(m_shooterSubsystem, () -> 0, () -> 0, () -> 0));

        NamedCommands.registerCommand("Shoot", new ShootCommand(m_indexerSubsystem));

        NamedCommands.registerCommand("PivotSeek",
                new PivotSeekCommand(m_pivotSubsystem, m_drivetrainSubsystem::getEstimatorPose3d));
    }
}
