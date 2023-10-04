package com.team2383.robot.commands;

import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WristPositionCommand extends CommandBase {
    private final WristSubsystem m_wrist;

    private final double m_angle;

    public WristPositionCommand(WristSubsystem wrist, double angleRad) {
        m_wrist = wrist;
        m_angle = angleRad;

        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        m_wrist.setPosition(m_angle);
    }

    @Override
    public boolean isFinished() {
        return m_wrist.isFinished();
    }
}
