// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.team2383.robot.Constants.Mode;
import com.team2383.robot.commands.drivetrain.DrivetrainHeadingCommand;
import com.team2383.robot.commands.drivetrain.FaceToTranslationCommand;
import com.team2383.robot.commands.drivetrain.JoystickDriveCommand;
import com.team2383.robot.commands.feeder.FeederPowerCommand;
import com.team2383.robot.commands.orchestra.OrchestraCommand;
import com.team2383.robot.commands.pivot.PivotPositionCommand;
import com.team2383.robot.commands.pivot.PivotPresets;
import com.team2383.robot.commands.shooter.ShooterRPMCommand;
import com.team2383.robot.commands.indexer.IndexerCommand;
import com.team2383.robot.subsystems.cameraSim.CameraSimSubsystem;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.drivetrain.GyroIO;
import com.team2383.robot.subsystems.drivetrain.GyroIOPigeon;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIO;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIOFalcon500;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIOSim;
import com.team2383.robot.subsystems.drivetrain.SLAM.SLAMConstantsConfig;
import com.team2383.robot.subsystems.feeder.FeederIO;
import com.team2383.robot.subsystems.feeder.FeederIONEO;
import com.team2383.robot.subsystems.feeder.FeederIOSim;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.indexer.IndexerIO;
import com.team2383.robot.subsystems.indexer.IndexerIONEO;
import com.team2383.robot.subsystems.indexer.IndexerIOSim;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.pivot.PivotIO;
import com.team2383.robot.subsystems.pivot.PivotIONeo;
import com.team2383.robot.subsystems.pivot.PivotIOSim;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterIO;
import com.team2383.robot.subsystems.shooter.ShooterIOFalcon500Neo;
import com.team2383.robot.subsystems.shooter.ShooterIOSim;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;
import com.team2383.robot.subsystems.sim_components.SimComponents;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    private final JoystickButton m_seek = new JoystickButton(m_driverController, 2);

    private final JoystickButton m_pivotZero = new JoystickButton(m_operatorController, 1);
    private final JoystickButton m_feedLeft = new JoystickButton(m_operatorController, 2);

    private DrivetrainSubsystem m_drivetrainSubsystem;
    private PivotSubsystem m_pivotSubsystem;
    private FeederSubsystem m_feederSubsystem;
    private IndexerSubsystem m_indexerSubsystem;
    private ShooterSubsystem m_shooterSubsystem;

    LoggedDashboardChooser<Boolean> enableLW = new LoggedDashboardChooser<Boolean>("Enable LW");

    LoggedDashboardNumber shooterTopBottomRPM = new LoggedDashboardNumber("Top Bottom RPM", 0);
    LoggedDashboardNumber shooterSideRPM = new LoggedDashboardNumber("Side RPM", 0);
    LoggedDashboardNumber shooterDifferentialRPM = new LoggedDashboardNumber("Differential RPM", 0);
    LoggedDashboardNumber pivotAngle = new LoggedDashboardNumber("Pivot Angle", 0);

    LoggedDashboardChooser<Command> testDashboardChooser = new LoggedDashboardChooser<Command>("Test Command");

    private boolean lwEnabled = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ROBOT_PROTO:
                    m_drivetrainSubsystem = new DrivetrainSubsystem(
                            new GyroIOPigeon(0, "Drive"),
                            new SwerveModuleIOFalcon500(DriveConstants.frontLeftConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.frontRightConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearLeftConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearRightConstants,
                                    Constants.kCANivoreBus));

                    m_pivotSubsystem = new PivotSubsystem(new PivotIONeo());

                    m_feederSubsystem = new FeederSubsystem(new FeederIONEO());

                    m_indexerSubsystem = new IndexerSubsystem(new IndexerIONEO());

                    m_shooterSubsystem = new ShooterSubsystem(new ShooterIOFalcon500Neo());
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

                    m_feederSubsystem = new FeederSubsystem(new FeederIOSim());

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

        m_feederSubsystem = m_feederSubsystem == null ? new FeederSubsystem(new FeederIO() {}) : m_feederSubsystem;

        m_indexerSubsystem = m_indexerSubsystem == null ? new IndexerSubsystem(new IndexerIO() {}) : m_indexerSubsystem;

        m_shooterSubsystem = m_shooterSubsystem == null ? new ShooterSubsystem(new ShooterIO() {}) : m_shooterSubsystem;

        new SimComponents(m_pivotSubsystem);

        configureDefaultCommands();

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
        m_seek.toggleOnTrue(new FaceToTranslationCommand(m_drivetrainSubsystem, new Translation2d(16.152, 5.5)));

        m_pivotZero.whileTrue(new PivotPositionCommand(m_pivotSubsystem, () -> PivotPresets.ZERO));
        m_feedLeft.whileTrue(new PivotPositionCommand(m_pivotSubsystem, () -> PivotPresets.FEED_LEFT));
    }

    private void configureDefaultCommands() {
        m_drivetrainSubsystem.setDefaultCommand(
                new JoystickDriveCommand(m_drivetrainSubsystem,
                        () -> new Translation2d(
                                MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveX),
                                        .1),
                                MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveY),
                                        .1)),
                        () -> Rotation2d
                                .fromDegrees(100 * MathUtil
                                        .applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveOmega), 0.1)),
                        () -> !(m_driverController.getRawButton(Constants.OI.FieldCentric)),
                        () -> -1));

        m_pivotSubsystem.setDefaultCommand(
                new PivotPositionCommand(m_pivotSubsystem,
                        () -> pivotAngle.get() * (Math.PI / 180)));

        m_feederSubsystem.setDefaultCommand(new FeederPowerCommand(m_feederSubsystem,
                () -> m_operatorController.getRawAxis(2) - m_operatorController.getRawAxis(3)));

        m_indexerSubsystem
                .setDefaultCommand(new IndexerCommand(m_indexerSubsystem, () -> m_operatorController.getRawAxis(1)));

        m_shooterSubsystem.setDefaultCommand(
                new ShooterRPMCommand(m_shooterSubsystem, shooterTopBottomRPM::get, shooterSideRPM::get,
                        shooterDifferentialRPM::get));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return (Command) null;
    }

    public Command getTestCommand() {
        return testDashboardChooser.get();
    }

    private void registerAutoCommands() {
    }

    private void registerTestCommands() {
        testDashboardChooser.addDefaultOption("None", (Command) null);

        testDashboardChooser.addOption("Drivetrain Dynamic Forward",
                m_drivetrainSubsystem.getDynamic(Direction.kForward));

        testDashboardChooser.addOption("Drivetrain Dynamic Reverse",
                m_drivetrainSubsystem.getDynamic(Direction.kReverse));

        testDashboardChooser.addOption("Drivetrain Quasistatic Forward",
                m_drivetrainSubsystem.getQuasiStatic(Direction.kForward));

        testDashboardChooser.addOption("Drivetrain Quasistatic Reverse",
                m_drivetrainSubsystem.getQuasiStatic(Direction.kReverse));

        testDashboardChooser.addOption("Shooter Dynamic Forward",
                m_shooterSubsystem.getDynamic(Direction.kForward));

        testDashboardChooser.addOption("Shooter Dynamic Reverse",
                m_shooterSubsystem.getDynamic(Direction.kReverse));

        testDashboardChooser.addOption("Shooter Quasistatic Forward",
                m_shooterSubsystem.getQuasiStatic(Direction.kForward));

        testDashboardChooser.addOption("Shooter Quasistatic Reverse",
                m_shooterSubsystem.getQuasiStatic(Direction.kReverse));

        testDashboardChooser.addOption("Sea Shanty 2", new OrchestraCommand("SeaShanty2.chrp"));
    }
}
