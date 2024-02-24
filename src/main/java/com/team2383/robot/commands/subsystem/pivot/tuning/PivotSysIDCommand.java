package com.team2383.robot.commands.subsystem.pivot.tuning;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class PivotSysIDCommand {
    private final SysIdRoutine sysid;

    private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> position = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

    public PivotSysIDCommand(PivotSubsystem pivot) {
        this.sysid = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> {
                            pivot.setVoltage(volts);
                        },
                        log -> {
                            pivot.periodic();

                            log.motor("Pivot")
                                    .voltage(voltage.mut_replace(pivot.getVoltage(), Volts))
                                    .angularPosition(position.mut_replace(pivot.getAngle(), Radians))
                                    .angularVelocity(
                                            velocity.mut_replace(pivot.getVelocity(), RadiansPerSecond));
                        }, pivot));
    }

    public Command getQuasiStatic(Direction direction) {
        return sysid.quasistatic(direction);
    }

    public Command getDynamic(Direction direction) {
        return sysid.dynamic(direction);
    }
}
