package com.team2383.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.team2383.robot.subsystems.feeder.FeederSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeederVoltageCommand extends CommandBase {
    private final FeederSubsystem feeder;
    private final DoubleSupplier power;
    private final BooleanSupplier cubeMode;

    // Negative power is intake, positive power is outtake
    public FeederVoltageCommand(FeederSubsystem feeder, DoubleSupplier power, Boolean cubeMode) {
        this(feeder, power, () -> cubeMode);
    }

    public FeederVoltageCommand(FeederSubsystem feeder, DoubleSupplier power, BooleanSupplier cubeMode) {
        this.power = power;
        this.feeder = feeder;
        this.cubeMode = cubeMode;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.setPower((power.getAsDouble() - 0.15) * (cubeMode.getAsBoolean() ? -0.5 : 1));
    }
}
