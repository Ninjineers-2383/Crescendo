package com.team2383.robot.commands;

import com.team2383.robot.subsystems.elevator.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorPositionCommand extends CommandBase {
    private final ElevatorSubsystem m_elevator;

    private final double m_position;

    public ElevatorPositionCommand(ElevatorSubsystem elevator, double position) {
        m_elevator = elevator;
        m_position = position;

        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.setPosition(m_position);
    }

    @Override
    public boolean isFinished() {
        return m_elevator.isFinished();
    }
}
