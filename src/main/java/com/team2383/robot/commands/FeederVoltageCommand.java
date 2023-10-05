package com.team2383.robot.commands;

import java.util.function.DoubleSupplier;

import com.team2383.robot.subsystems.feeder.FeederSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeederVoltageCommand extends CommandBase {
    private final FeederSubsystem feeder;
    private final DoubleSupplier power;

    public FeederVoltageCommand(FeederSubsystem feeder, DoubleSupplier power) {
        this.power = power;
        this.feeder = feeder;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.setPower(power.getAsDouble());
    }
}
