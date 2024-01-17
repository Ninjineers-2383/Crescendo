// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.team2383.robot.Constants.Mode;
import com.team2383.robot.commands.IndexerCommand;
import com.team2383.robot.commands.ShooterRPMCommand;
import com.team2383.robot.subsystems.indexer.IndexerIONEO;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterIOFalcon500;
import com.team2383.robot.subsystems.shooter.ShooterIOSim;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

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
    // private final GenericHID m_operatorController = new GenericHID(1);

    // private DrivetrainSubsystem m_drivetrainSubsystem;

    // private FeederSubsystem m_leftFeederSubsystem;
    // private FeederSubsystem m_rightFeederSubsystem;

    private IndexerSubsystem m_indexerSubsystem;

    // private PivotSubsystem m_pivotSubsystem;

    private ShooterSubsystem m_shooterSubsystem;

    // private FeederSubsystem m_feederSubsystem;

    LoggedDashboardChooser<Boolean> enableLW = new LoggedDashboardChooser<Boolean>("Enable LW");

    LoggedDashboardNumber shooterRPM = new LoggedDashboardNumber("RPM", 0);

    LoggedDashboardChooser<Command> sysIDashboardChooser = new LoggedDashboardChooser<Command>("SysID");

    private boolean lwEnabled = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ROBOT_PROTO:
                    // m_drivetrainSubsystem = new DrivetrainSubsystem(
                    // new GyroIO() {},
                    // new SwerveModuleIOFalcon500(DriveConstants.frontLeftConstants,
                    // Constants.kCANivoreBus),
                    // new SwerveModuleIOFalcon500(DriveConstants.frontRightConstants,
                    // Constants.kCANivoreBus),
                    // new SwerveModuleIOFalcon500(DriveConstants.rearLeftConstants,
                    // Constants.kCANivoreBus),
                    // new SwerveModuleIOFalcon500(DriveConstants.rearRightConstants,
                    // Constants.kCANivoreBus));

                    // m_leftFeederSubsystem = new FeederSubsystem(new
                    // FeederIOFalcon500(FeederConstants.kLeftMotorID));
                    // m_rightFeederSubsystem = new FeederSubsystem(new
                    // FeederIOFalcon500(FeederConstants.kRightMotorID));

                    m_indexerSubsystem = new IndexerSubsystem(new IndexerIONEO());

                    // m_pivotSubsystem = new PivotSubsystem(new PivotIOFalcon500());

                    m_shooterSubsystem = new ShooterSubsystem(new ShooterIOFalcon500());
                    // m_feederSubsystem = new FeederSubsystem(new FeederIONEO());
                    break;
                case ROBOT_SIM:
                    // m_drivetrainSubsystem = new DrivetrainSubsystem(
                    // new GyroIO() {},
                    // new SwerveModuleIOSim(), new SwerveModuleIOSim(),
                    // new SwerveModuleIOSim(), new SwerveModuleIOSim());

                    // new CameraSimSubsystem("northstar-1", SLAMConstantsConfig.camTransforms[0],
                    // m_drivetrainSubsystem::getDeadReckoningPose3d);
                    // new CameraSimSubsystem("northstar-2", SLAMConstantsConfig.camTransforms[1],
                    // m_drivetrainSubsystem::getDeadReckoningPose3d);
                    // new CameraSimSubsystem("northstar-3", SLAMConstantsConfig.camTransforms[2],
                    // m_drivetrainSubsystem::getDeadReckoningPose3d);
                    // new CameraSimSubsystem("northstar-4", SLAMConstantsConfig.camTransforms[3],
                    // m_drivetrainSubsystem::getDeadReckoningPose3d);

                    // m_leftFeederSubsystem = new FeederSubsystem(new FeederIOSim());
                    // m_rightFeederSubsystem = new FeederSubsystem(new FeederIOSim());

                    // m_indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());

                    // m_pivotSubsystem = new PivotSubsystem(new PivotIOSim());

                    m_shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
                    break;
                default:
                    break;
            }
        }

        // m_drivetrainSubsystem = m_drivetrainSubsystem != null ? m_drivetrainSubsystem
        // : new DrivetrainSubsystem(
        // new GyroIO() {},
        // new SwerveModuleIO() {}, new SwerveModuleIO() {},
        // new SwerveModuleIO() {}, new SwerveModuleIO() {});

        configureDefaultCommands();
        registerAutoCommands();
        // Configure the button bindings
        configureButtonBindings();

        enableLW.addDefaultOption("No", false);
        enableLW.addOption("Yes", true);

        sysIDashboardChooser.addDefaultOption("Quasi Static", m_shooterSubsystem.getQuasiStatic(Direction.kForward));
        sysIDashboardChooser.addDefaultOption("Dynamic", m_shooterSubsystem.getDynamic(Direction.kForward));

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
        // new JoystickButton(m_driverController, 1)
        // .toggleOnTrue(new JoystickDriveHeadingLock(m_drivetrainSubsystem,
        // () -> new Translation2d(
        // MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveX) *
        // 0.5, .1),
        // MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveY) *
        // 0.5, .1)),
        // () -> Rotation2d
        // .fromDegrees(100 * MathUtil
        // .applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveOmega) * 0.5,
        // 0.1)),
        // () -> !(m_driverController.getRawButton(Constants.OI.FieldCentric)),
        // () -> -1));

        // new JoystickButton(m_driverController, 8)
        // .onTrue(new InstantCommand(() -> {
        // m_drivetrainSubsystem.forceHeading(new Rotation2d());
        // }));

        new JoystickButton(m_driverController, 3)
                .whileTrue(new ShooterRPMCommand(m_shooterSubsystem, () -> 1000));

        new JoystickButton(m_driverController, 4)
                .whileTrue(new IndexerCommand(m_indexerSubsystem, () -> 1));

    }

    private void configureDefaultCommands() {
        m_shooterSubsystem.setDefaultCommand(new ShooterRPMCommand(m_shooterSubsystem, shooterRPM::get));
        m_indexerSubsystem.setDefaultCommand(new IndexerCommand(m_indexerSubsystem, () -> 0));
        // m_feederSubsystem.setDefaultCommand(new FeederPowerCommand(m_feederSubsystem,
        // () -> m_driverController.getRawAxis(3) - m_driverController.getRawAxis(2)));
        // m_drivetrainSubsystem.setDefaultCommand(
        // new JoystickDriveHeadingLock(m_drivetrainSubsystem,
        // () -> new Translation2d(
        // MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveX),
        // .1),
        // MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveY),
        // .1)),
        // () -> Rotation2d
        // .fromDegrees(100 * MathUtil
        // .applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveOmega), 0.1)),
        // () -> !(m_driverController.getRawButton(Constants.OI.FieldCentric)),
        // () -> -1));

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
        return sysIDashboardChooser.get();
    }

    private void registerAutoCommands() {
    }
}
