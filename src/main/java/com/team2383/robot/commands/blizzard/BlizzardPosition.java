package com.team2383.robot.commands.blizzard;

import edu.wpi.first.math.geometry.Rotation2d;

public class BlizzardPosition {
    private double m_extensionLength;

    private Rotation2d m_wristAngle;

    public BlizzardPosition(double extensionLength, Rotation2d wristAngle) {
        m_extensionLength = extensionLength;
        m_wristAngle = wristAngle;

    }

    public double getExtension() {
        return m_extensionLength;
    }

    public Rotation2d getWristAngle() {
        return m_wristAngle;
    }
}
