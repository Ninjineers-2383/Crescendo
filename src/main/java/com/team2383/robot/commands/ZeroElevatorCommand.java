package com.team2383.robot.commands;

import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroElevatorCommand extends CommandBase {

    private ElevatorSubsystem m_elevator;

    private int count = 0;

    public ZeroElevatorCommand(ElevatorSubsystem elevator) {
        m_elevator = elevator;

        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.setPosition(-10);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.getPosition();
        m_elevator.setPosition(0);
    }

    @Override
    public boolean isFinished() {
        if (m_elevator.getCurrent() > 18) {
            count++;
        } else {
            count = 0;
        }
        return count > 7;
    }
}
