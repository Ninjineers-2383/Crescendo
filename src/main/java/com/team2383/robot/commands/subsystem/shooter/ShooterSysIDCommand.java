package com.team2383.robot.commands.subsystem.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ShooterSysIDCommand {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Angle> m_distance = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine;

    public ShooterSysIDCommand(ShooterSubsystem shooter) {
        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motors.
                        (Measure<Voltage> volts) -> {
                            shooter.setTopBottomVoltage(volts.in(Volts));
                            shooter.setSideVoltage(volts.in(Volts));
                        },
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being
                        // characterized.
                        log -> {
                            // Record a frame for the left motors. Since these share an encoder, we consider
                            // the entire group to be one motor.
                            shooter.periodic();
                            log.motor("top-shooter")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    shooter.getVoltages()[0], Volts))
                                    .angularPosition(m_distance.mut_replace(shooter.getPositions()[0], Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(shooter.getVelocities()[0], RotationsPerSecond));
                            log.motor("bottom-shooter")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    shooter.getVoltages()[1], Volts))
                                    .angularPosition(m_distance.mut_replace(shooter.getPositions()[1], Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(shooter.getVelocities()[1], RotationsPerSecond));
                            log.motor("side-shooter")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    shooter.getVoltages()[2], Volts))
                                    .angularPosition(m_distance.mut_replace(shooter.getPositions()[2], Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(shooter.getVelocities()[2], RotationsPerSecond));
                        },
                        // Tell SysId to make generated commands require this subsystem, suffix test
                        // state in
                        // WPILog with this subsystem's name
                        shooter));
    }

    public Command getQuasiStatic(Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command getDynamic(Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
