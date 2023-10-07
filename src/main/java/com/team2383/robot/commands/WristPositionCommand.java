package com.team2383.robot.commands;

import com.team2383.robot.subsystems.wrist.WristSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WristPositionCommand extends CommandBase {
    private final WristSubsystem m_wrist;

    private final Rotation2d m_angle;

    public WristPositionCommand(WristSubsystem wrist, Rotation2d angleRad) {
        m_wrist = wrist;
        m_angle = angleRad;

        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        m_wrist.setPosition(m_angle.getRadians());
    }

    @Override
    public boolean isFinished() {
        return m_wrist.isFinished();
    }
}
