package com.team2383.robot.commands.subsystem.trap_feeder;

import com.team2383.robot.subsystems.trap_feeder.TrapFeederSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class TrapFeederPowerCommand extends Command {
    private final TrapFeederSubsystem trapFeeder;
    private final double power;

    public TrapFeederPowerCommand(TrapFeederSubsystem trapFeeder, double power) {
        this.trapFeeder = trapFeeder;
        this.power = power;

        addRequirements(trapFeeder);
    }

    @Override
    public void execute() {
        trapFeeder.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        trapFeeder.setPower(0);
    }
}
