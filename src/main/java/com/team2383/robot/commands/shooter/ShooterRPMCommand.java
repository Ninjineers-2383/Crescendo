package com.team2383.robot.commands.shooter;

import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterRPMCommand extends Command {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier topBottomRPM;
    private final DoubleSupplier sideRPM;

    public ShooterRPMCommand(ShooterSubsystem shooter, DoubleSupplier topBottomRPM, DoubleSupplier sideRPM) {
        this.shooter = shooter;
        this.topBottomRPM = topBottomRPM;
        this.sideRPM = sideRPM;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setTopBottomRPM(topBottomRPM.getAsDouble());
        shooter.setSideRPM(sideRPM.getAsDouble());
    }
}
