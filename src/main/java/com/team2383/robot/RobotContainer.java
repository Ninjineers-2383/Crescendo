// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.robot;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.robot.Constants.Mode;
import com.team2383.robot.autos.CubeMobilityAuto;
import com.team2383.robot.commands.ElevatorPositionCommand;
import com.team2383.robot.commands.ElevatorVelocityCommand;
import com.team2383.robot.commands.FeederVoltageCommand;
import com.team2383.robot.commands.JoystickDriveHeadingLock;
import com.team2383.robot.commands.WristVelocityCommand;
import com.team2383.robot.commands.blizzard.BlizzardCommand;
import com.team2383.robot.commands.blizzard.BlizzardPresets;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.drivetrain.GyroIO;
import com.team2383.robot.subsystems.drivetrain.GyroIONavX;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIO;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIOFalcon500;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIOSim;
import com.team2383.robot.subsystems.drivetrain.vision.VisionIO;
import com.team2383.robot.subsystems.drivetrain.vision.VisionIOPhoton;
import com.team2383.robot.subsystems.elevator.ElevatorIO;
import com.team2383.robot.subsystems.elevator.ElevatorIOFalcon500;
import com.team2383.robot.subsystems.elevator.ElevatorIOSim;
import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;
import com.team2383.robot.subsystems.feeder.FeederIO;
import com.team2383.robot.subsystems.feeder.FeederIOFalcon500;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.sim_components.SimComponents;
import com.team2383.robot.subsystems.wrist.WristIO;
import com.team2383.robot.subsystems.wrist.WristIOSim;
import com.team2383.robot.subsystems.wrist.WristIOSparkMax;
import com.team2383.robot.subsystems.wrist.WristSubsystem;

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
import edu.wpi.first.wpilibj2.command.button.POVButton;

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

    private DrivetrainSubsystem m_drivetrainSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    private WristSubsystem m_wristSubsystem;
    private FeederSubsystem m_feederSubsystem;

    LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<Command>("Auto");
    LoggedDashboardChooser<Boolean> enableLW = new LoggedDashboardChooser<Boolean>("Enable LW");

    private boolean lwEnabled = false;

    public SwerveAutoBuilder autoBuilder;
    HashMap<String, Command> autoHashMap;

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
                    m_elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOFalcon500(Constants.kCANivoreBus));
                    m_wristSubsystem = new WristSubsystem(new WristIOSparkMax());
                    m_feederSubsystem = new FeederSubsystem(new FeederIOFalcon500());
                    break;
                case ROBOT_SIM:
                    m_drivetrainSubsystem = new DrivetrainSubsystem(
                            new GyroIO() {},
                            new VisionIO() {},
                            new SwerveModuleIOSim(), new SwerveModuleIOSim(),
                            new SwerveModuleIOSim(), new SwerveModuleIOSim());
                    m_elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim() {});
                    m_wristSubsystem = new WristSubsystem(new WristIOSim());
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

        m_wristSubsystem = m_wristSubsystem != null ? m_wristSubsystem
                : new WristSubsystem(new WristIO() {});

        m_feederSubsystem = m_feederSubsystem != null ? m_feederSubsystem
                : new FeederSubsystem(new FeederIO() {});

        new SimComponents(m_elevatorSubsystem, m_wristSubsystem);

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
        // Ground Intake
        new JoystickButton(m_operatorController, 1)
                .onTrue(new BlizzardCommand(m_elevatorSubsystem, m_wristSubsystem, BlizzardPresets.GROUND_INTAKE));

        // Cone Chute
        new JoystickButton(m_operatorController, 2)
                .onTrue(new BlizzardCommand(m_elevatorSubsystem, m_wristSubsystem, BlizzardPresets.CONE_CHUTE));
        // Middle
        new JoystickButton(m_operatorController, 3)
                .onTrue(new BlizzardCommand(m_elevatorSubsystem, m_wristSubsystem, BlizzardPresets.MIDDLE));
        // High
        new JoystickButton(m_operatorController, 4)
                .onTrue(new BlizzardCommand(m_elevatorSubsystem, m_wristSubsystem, BlizzardPresets.HIGH));

        new POVButton(m_operatorController, 0)
                .onTrue(new BlizzardCommand(m_elevatorSubsystem, m_wristSubsystem, BlizzardPresets.SLIDER));

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

        m_feederSubsystem.setDefaultCommand(new FeederVoltageCommand(m_feederSubsystem,
                () -> m_driverController.getRawAxis(2) - m_driverController.getRawAxis(3)));

        m_wristSubsystem.setDefaultCommand(
                new WristVelocityCommand(m_wristSubsystem,
                        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.02)));

        m_elevatorSubsystem.setDefaultCommand(
                new ElevatorVelocityCommand(m_elevatorSubsystem,
                        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(5), 0.02)));
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

                put("Cube", new BlizzardCommand(m_elevatorSubsystem, m_wristSubsystem, BlizzardPresets.HIGH));
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
        autoChooser.addOption("Cube Mobility", new CubeMobilityAuto(m_drivetrainSubsystem, m_elevatorSubsystem,
                m_wristSubsystem, "Path1", autoBuilder));
    }
}
