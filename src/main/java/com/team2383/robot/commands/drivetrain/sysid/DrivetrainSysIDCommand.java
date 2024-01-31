package com.team2383.robot.commands.drivetrain.sysid;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import static edu.wpi.first.units.MutableMeasure.mutable;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class DrivetrainSysIDCommand {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Distance> m_distanceDrive = mutable(Meters.of(0));

    private final MutableMeasure<Angle> m_distanceAngle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocityDrive = mutable(MetersPerSecond.of(0));

    private final MutableMeasure<Velocity<Angle>> m_velocityAngle = mutable(RotationsPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine;

    public DrivetrainSysIDCommand(DrivetrainSubsystem drivetrain) {
        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motors.
                        (Measure<Voltage> volts) -> {
                            drivetrain.setModuleVoltages(volts);
                        },
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being
                        // characterized.
                        log -> {
                            drivetrain.periodic();

                            log.motor("Front Left Drive")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    drivetrain.getModules()[0].m_inputs.appliedVoltsDrive, Volts))
                                    .linearPosition(
                                            m_distanceDrive.mut_replace(
                                                    drivetrain.getModules()[0].m_inputs.drivePositionM,
                                                    Meters))
                                    .linearVelocity(
                                            m_velocityDrive.mut_replace(
                                                    drivetrain.getModules()[0].m_inputs.driveVelocityMPS,
                                                    MetersPerSecond));

                            log.motor("Front Left Azimuth")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    drivetrain.getModules()[0].m_inputs.appliedVoltsAngle, Volts))
                                    .angularPosition(
                                            m_distanceAngle.mut_replace(drivetrain.getModules()[0].m_inputs.angleRad,
                                                    Radians))
                                    .angularVelocity(
                                            m_velocityAngle.mut_replace(
                                                    drivetrain.getModules()[0].m_inputs.azimuthVelocityRPM / 60.0,
                                                    RotationsPerSecond));

                            log.motor("Front Right Drive")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    drivetrain.getModules()[1].m_inputs.appliedVoltsDrive, Volts))
                                    .linearPosition(
                                            m_distanceDrive.mut_replace(
                                                    drivetrain.getModules()[1].m_inputs.drivePositionM,
                                                    Meters))
                                    .linearVelocity(
                                            m_velocityDrive.mut_replace(
                                                    drivetrain.getModules()[1].m_inputs.driveVelocityMPS,
                                                    MetersPerSecond));

                            log.motor("Front Right Azimuth")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    drivetrain.getModules()[1].m_inputs.appliedVoltsAngle, Volts))
                                    .angularPosition(
                                            m_distanceAngle.mut_replace(drivetrain.getModules()[1].m_inputs.angleRad,
                                                    Radians))
                                    .angularVelocity(
                                            m_velocityAngle.mut_replace(
                                                    drivetrain.getModules()[1].m_inputs.azimuthVelocityRPM / 60.0,
                                                    RotationsPerSecond));

                            log.motor("Rear Left Drive")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    drivetrain.getModules()[2].m_inputs.appliedVoltsDrive, Volts))
                                    .linearPosition(
                                            m_distanceDrive.mut_replace(
                                                    drivetrain.getModules()[2].m_inputs.drivePositionM,
                                                    Meters))
                                    .linearVelocity(
                                            m_velocityDrive.mut_replace(
                                                    drivetrain.getModules()[2].m_inputs.driveVelocityMPS,
                                                    MetersPerSecond));

                            log.motor("Rear Left Azimuth")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    drivetrain.getModules()[2].m_inputs.appliedVoltsAngle, Volts))
                                    .angularPosition(
                                            m_distanceAngle.mut_replace(drivetrain.getModules()[2].m_inputs.angleRad,
                                                    Radians))
                                    .angularVelocity(
                                            m_velocityAngle.mut_replace(
                                                    drivetrain.getModules()[2].m_inputs.azimuthVelocityRPM / 60.0,
                                                    RotationsPerSecond));

                            log.motor("Rear Right Drive")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    drivetrain.getModules()[3].m_inputs.appliedVoltsDrive, Volts))
                                    .linearPosition(
                                            m_distanceDrive.mut_replace(
                                                    drivetrain.getModules()[3].m_inputs.drivePositionM,
                                                    Meters))
                                    .linearVelocity(
                                            m_velocityDrive.mut_replace(
                                                    drivetrain.getModules()[3].m_inputs.driveVelocityMPS,
                                                    MetersPerSecond));

                            log.motor("Rear Right Azimuth")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    drivetrain.getModules()[3].m_inputs.appliedVoltsAngle, Volts))
                                    .angularPosition(
                                            m_distanceAngle.mut_replace(drivetrain.getModules()[3].m_inputs.angleRad,
                                                    Radians))
                                    .angularVelocity(
                                            m_velocityAngle.mut_replace(
                                                    drivetrain.getModules()[3].m_inputs.azimuthVelocityRPM / 60.0,
                                                    RotationsPerSecond));
                        },
                        // Tell SysId to make generated commands require this subsystem, suffix test
                        // state in
                        // WPILog with this subsystem's name
                        drivetrain));
    }

    public Command getQuasiStatic(Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command getDynamic(Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
