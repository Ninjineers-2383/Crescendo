package com.team2383.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO shooter;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Angle> m_distance = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine;

    public ShooterSubsystem(ShooterIO io) {
        shooter = io;

        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motors.
                        (Measure<Voltage> volts) -> {
                            shooter.setVoltage(volts.in(Volts));
                        },
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being
                        // characterized.
                        log -> {
                            // Record a frame for the left motors. Since these share an encoder, we consider
                            // the entire group to be one motor.
                            shooter.updateInputs(inputs);
                            log.motor("left-shooter")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    inputs.leftVoltage, Volts))
                                    .angularPosition(m_distance.mut_replace(inputs.leftPosition, Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(inputs.leftVelocity, RotationsPerSecond));
                            log.motor("right-shooter")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    inputs.rightVoltage, Volts))
                                    .angularPosition(m_distance.mut_replace(inputs.rightPosition, Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(inputs.rightVelocity, RotationsPerSecond));
                        },
                        // Tell SysId to make generated commands require this subsystem, suffix test
                        // state in
                        // WPILog with this subsystem's name
                        this));
    }

    @Override
    public void periodic() {
        shooter.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setRPM(double RPM) {
        shooter.setRPM(RPM);
    }

    public Command getQuasiStatic(Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command getDynamic(Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
