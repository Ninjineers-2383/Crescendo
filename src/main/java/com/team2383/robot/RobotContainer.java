// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.team2383.robot.Constants.Mode;
import com.team2383.robot.commands.JoystickDriveHeadingLock;
import com.team2383.robot.subsystems.cameraSim.CameraSimSubsystem;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.drivetrain.GyroIO;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIO;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIOFalcon500;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIOSim;
import com.team2383.robot.subsystems.drivetrain.SLAM.SLAMConstantsConfig;
import com.team2383.robot.subsystems.gamePieceSim.GamePieceSimSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

    private DrivetrainSubsystem m_drivetrainSubsystem;

    private GamePieceSimSubsystem m_gamePieceSimSubsystem;

    // private final AutoChooser autoChooser;
    // private final TeleOpChooser teleOpChooser;

    LoggedDashboardChooser<Boolean> enableLW = new LoggedDashboardChooser<Boolean>("Enable LW");

    LoggedDashboardChooser<Boolean> shootingBoolean = new LoggedDashboardChooser<Boolean>("Shooting");

    LoggedDashboardChooser<Boolean> intakeBoolean = new LoggedDashboardChooser<Boolean>("Intake");

    LoggedDashboardNumber shooterAngle = new LoggedDashboardNumber("Shooter Angle", 45);

    LoggedDashboardNumber shooterRPM = new LoggedDashboardNumber("RPM", 1000);

    LoggedDashboardChooser<Integer> pieceIndexChooser = new LoggedDashboardChooser<Integer>("Piece Index");

    private boolean lwEnabled = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ROBOT_COMP:
                    m_drivetrainSubsystem = new DrivetrainSubsystem(
                            new GyroIO() {},
                            new SwerveModuleIOFalcon500(DriveConstants.frontLeftConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.frontRightConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearLeftConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearRightConstants,
                                    Constants.kCANivoreBus));

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

                    m_gamePieceSimSubsystem = new GamePieceSimSubsystem(m_drivetrainSubsystem::getDeadReckoningPose3d,
                            m_drivetrainSubsystem::getRobotRelativeSpeeds, shootingBoolean::get, intakeBoolean::get,
                            shooterAngle::get, shooterRPM::get);
                    break;
                default:
                    break;
            }
        }

        m_drivetrainSubsystem = m_drivetrainSubsystem != null ? m_drivetrainSubsystem
                : new DrivetrainSubsystem(
                        new GyroIO() {},
                        new SwerveModuleIO() {}, new SwerveModuleIO() {},
                        new SwerveModuleIO() {}, new SwerveModuleIO() {});

        configureDefaultCommands();

        registerAutoCommands();

        // Configure the button bindings
        configureButtonBindings();

        enableLW.addDefaultOption("No", false);
        enableLW.addOption("Yes", true);

        shootingBoolean.addDefaultOption("No", false);
        shootingBoolean.addOption("Yes", true);

        intakeBoolean.addDefaultOption("No", false);
        intakeBoolean.addOption("Yes", true);

        pieceIndexChooser.addDefaultOption("0", 0);
        pieceIndexChooser.addOption("1", 1);
        pieceIndexChooser.addOption("2", 2);
        pieceIndexChooser.addOption("3", 3);
        pieceIndexChooser.addOption("4", 4);
        pieceIndexChooser.addOption("5", 5);
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
        new JoystickButton(m_driverController, 1)
                .toggleOnTrue(new JoystickDriveHeadingLock(m_drivetrainSubsystem,
                        () -> new Translation2d(
                                MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveX) * 0.5, .1),
                                MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveY) * 0.5, .1)),
                        () -> Rotation2d
                                .fromDegrees(100 * MathUtil
                                        .applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveOmega) * 0.5,
                                                0.1)),
                        () -> !(m_driverController.getRawButton(Constants.OI.FieldCentric)),
                        () -> -1));

        new JoystickButton(m_driverController, 8)
                .onTrue(new InstantCommand(() -> {
                    m_drivetrainSubsystem.forceHeading(new Rotation2d());
                }));

    }

    private void configureDefaultCommands() {
        m_drivetrainSubsystem.setDefaultCommand(
                new JoystickDriveHeadingLock(m_drivetrainSubsystem,
                        () -> new Translation2d(
                                MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveX), .1),
                                MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveY), .1)),
                        () -> Rotation2d
                                .fromDegrees(100 * MathUtil
                                        .applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveOmega), 0.1)),
                        () -> !(m_driverController.getRawButton(Constants.OI.FieldCentric)),
                        () -> -1));

        m_gamePieceSimSubsystem.setDefaultCommand(new InstantCommand(() -> {
            m_gamePieceSimSubsystem.movePiece(() -> m_driverController.getRawAxis(3),
                    () -> m_driverController.getRawAxis(4), () -> m_driverController.getRawAxis(5),
                    pieceIndexChooser::get);
        }, m_gamePieceSimSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return (Command) null;
    }

    private void registerAutoCommands() {
    }
}
