// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.team2383.robot.Constants.Mode;
import com.team2383.robot.autos.auto_blocks.Engage;
import com.team2383.robot.commands.JoystickDriveHeadingLock;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.drivetrain.GyroIO;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIO;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIOFalcon500;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIOSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private Boolean cubeMode = true;

    // private final AutoChooser autoChooser;
    // private final TeleOpChooser teleOpChooser;

    LoggedDashboardChooser<Boolean> enableLW = new LoggedDashboardChooser<Boolean>("Enable LW");

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
        registerAutoCommands(); // Configure the button bindings
        configureButtonBindings();

        enableLW.addDefaultOption("No", false);
        enableLW.addOption("Yes", true);

    }

    public void periodic() {
        SmartDashboard.putBoolean("Cube Mode", cubeMode);

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

        new JoystickButton(m_driverController, 3).onTrue(new InstantCommand(() -> {
            cubeMode = true;
        }));
        new JoystickButton(m_driverController, 4).onTrue(new InstantCommand(() -> {
            cubeMode = false;
        }));

        new JoystickButton(m_driverController, 8)
                .onTrue(new InstantCommand(() -> {
                    m_drivetrainSubsystem.resetHeading();
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
        NamedCommands.registerCommand("Engage", new Engage(m_drivetrainSubsystem, true));
    }
}
