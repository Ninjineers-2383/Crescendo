package com.team2383.robot.commands;

import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterRPMCommand extends Command {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier RPM;

    public ShooterRPMCommand(ShooterSubsystem shooter, DoubleSupplier RPM) {
        this.shooter = shooter;
        this.RPM = RPM;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setRPM(RPM.getAsDouble());
    }
}
