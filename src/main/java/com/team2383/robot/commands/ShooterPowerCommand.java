package com.team2383.robot.commands;

import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterPowerCommand extends Command {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier power;

    public ShooterPowerCommand(ShooterSubsystem shooter, DoubleSupplier power) {
        this.shooter = shooter;
        this.power = power;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setPower(power.getAsDouble());
    }
}
