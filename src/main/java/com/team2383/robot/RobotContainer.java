// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.robot;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.robot.Constants.Mode;
import com.team2383.robot.commands.ElevatorPositionCommand;
import com.team2383.robot.commands.JoystickDriveHeadingLock;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.drivetrain.GyroIO;
import com.team2383.robot.subsystems.drivetrain.GyroIONavX;
import com.team2383.robot.subsystems.drivetrain.GyroIOPigeon;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIO;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIOFalcon500;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIOSim;
import com.team2383.robot.subsystems.drivetrain.vision.VisionIO;
import com.team2383.robot.subsystems.drivetrain.vision.VisionIOPhoton;
import com.team2383.robot.subsystems.elevator.ElevatorIO;
import com.team2383.robot.subsystems.elevator.ElevatorIOSim;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.sim_components.SimComponents;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

    private DrivetrainSubsystem m_drivetrainSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;

    LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<Command>("Auto");
    LoggedDashboardChooser<Boolean> enableLW = new LoggedDashboardChooser<Boolean>("Enable LW");

    private boolean lwEnabled = false;

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> autoHashMap;

    public SwerveAutoBuilder autoBuilder;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ROBOT_COMP:
                    m_drivetrainSubsystem = new DrivetrainSubsystem(
                            new GyroIONavX(),
                            new VisionIOPhoton(),
                            new SwerveModuleIOFalcon500(DriveConstants.frontLeftConstants,
                                    DriveConstants.frontLeftEncoder, Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.frontRightConstants,
                                    DriveConstants.frontRightEncoder, Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearLeftConstants,
                                    DriveConstants.rearLeftEncoder, Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearRightConstants,
                                    DriveConstants.rearRightEncoder, Constants.kCANivoreBus));
                    break;
                case ROBOT_SIM:
                    m_drivetrainSubsystem = new DrivetrainSubsystem(
                            new GyroIO() {},
                            new VisionIO() {},
                            new SwerveModuleIOSim(), new SwerveModuleIOSim(),
                            new SwerveModuleIOSim(), new SwerveModuleIOSim());
                    m_elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim() {});
                    break;
                default:
                    break;
            }
        }

        m_drivetrainSubsystem = m_drivetrainSubsystem != null ? m_drivetrainSubsystem
                : new DrivetrainSubsystem(
                        new GyroIO() {},
                        new VisionIO() {},
                        new SwerveModuleIO() {}, new SwerveModuleIO() {},
                        new SwerveModuleIO() {}, new SwerveModuleIO() {});

        m_elevatorSubsystem = m_elevatorSubsystem != null ? m_elevatorSubsystem
                : new ElevatorSubsystem(new ElevatorIO() {});

        new SimComponents(m_elevatorSubsystem);

        // Configure the button bindings
        configureButtonBindings();
        configureDefaultCommands();
        setAutoCommands();

        enableLW.addDefaultOption("No", false);
        enableLW.addOption("Yes", true);
    }

    public void periodic() {
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
                .onTrue(new ElevatorPositionCommand(m_elevatorSubsystem, Units.inchesToMeters(0)));
        new JoystickButton(m_driverController, 2)
                .onTrue(new ElevatorPositionCommand(m_elevatorSubsystem, Units.inchesToMeters(10)));
        new JoystickButton(m_driverController, 3)
                .onTrue(new ElevatorPositionCommand(m_elevatorSubsystem, Units.inchesToMeters(20)));
        new JoystickButton(m_driverController, 4)
                .onTrue(new ElevatorPositionCommand(m_elevatorSubsystem, Units.inchesToMeters(45)));

        new JoystickButton(m_driverController, 8).onTrue(new InstantCommand(() -> {
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
        return autoChooser.get();
    }

    private void setAutoCommands() {
        autoHashMap = new HashMap<>() {
            {
                put("Auto Log", new PrintCommand("Auto Event: log"));
            }
        };

        autoBuilder = new SwerveAutoBuilder(
                m_drivetrainSubsystem::getPose,
                m_drivetrainSubsystem::forceOdometry,
                m_drivetrainSubsystem.m_kinematics,
                new PIDConstants(5, 0, 0),
                new PIDConstants(4, 0, 0),
                m_drivetrainSubsystem::setModuleStates,
                autoHashMap,
                true,
                m_drivetrainSubsystem);

        Command nullAuto = null;

        autoChooser.addDefaultOption("No Auto :(", nullAuto);
        autoChooser.addOption("5 in", new ElevatorPositionCommand(m_elevatorSubsystem, Units.inchesToMeters(5)));
        autoChooser.addOption("10 in", new ElevatorPositionCommand(m_elevatorSubsystem, Units.inchesToMeters(10)));
    }
}
