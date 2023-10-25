package com.team2383.robot.commands;

import java.util.function.DoubleSupplier;

import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class WristVelocityCommand extends Command {
    private final WristSubsystem wrist;
    private final DoubleSupplier power;

    public WristVelocityCommand(WristSubsystem wrist, DoubleSupplier power) {
        this.wrist = wrist;
        this.power = power;

        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.setVelocity(power.getAsDouble());
    }
}
