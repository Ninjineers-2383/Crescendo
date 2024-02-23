package com.team2383.robot.commands.subsystem.feeder;

import com.team2383.robot.subsystems.feeder.FeederSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class FeederPowerCommand extends Command {
    private final FeederSubsystem feeder;
    private final DoubleSupplier power;

    public FeederPowerCommand(FeederSubsystem feeder, DoubleSupplier power) {
        this.feeder = feeder;
        this.power = power;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.setPower(power.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
