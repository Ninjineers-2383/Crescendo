package com.team2383.robot.commands.subsystem.shooter;

import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterRPMCommand extends Command {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier topBottomRPM;
    private final DoubleSupplier sideRPM;
    private final DoubleSupplier differentialRPM;

    public ShooterRPMCommand(ShooterSubsystem shooter, DoubleSupplier topBottomRPM, DoubleSupplier sideRPM,
            DoubleSupplier differentialRPM) {
        this.shooter = shooter;
        this.topBottomRPM = topBottomRPM;
        this.sideRPM = sideRPM;
        this.differentialRPM = differentialRPM;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setTopBottomRPM(topBottomRPM.getAsDouble(), differentialRPM.getAsDouble());
        shooter.setSideRPM(sideRPM.getAsDouble());
    }

    // @Override
    // public boolean isFinished() {
    // return Math.abs(shooter.getTopBottomRPM() - topBottomRPM.getAsDouble()) <
    // 0.5;
    // }
}
