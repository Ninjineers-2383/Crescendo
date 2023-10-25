package com.team2383.robot.commands;

import java.util.function.DoubleSupplier;

import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorVelocityCommand extends Command {
    private ElevatorSubsystem elevator;
    private DoubleSupplier power;

    public ElevatorVelocityCommand(ElevatorSubsystem elevator, DoubleSupplier voltage) {
        power = voltage;
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setVelocity(power.getAsDouble());
    }
}
