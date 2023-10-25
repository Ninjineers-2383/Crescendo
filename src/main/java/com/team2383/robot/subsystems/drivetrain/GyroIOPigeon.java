package com.team2383.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOPigeon implements GyroIO {
    private final Pigeon2 m_gyro;

    public GyroIOPigeon(int id, String CANbus) {
        this.m_gyro = new Pigeon2(id, CANbus);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true; // TODO: Figure out how to get connected status
        inputs.headingDeg = m_gyro.getYaw().getValue();
        inputs.rollDeg = m_gyro.getRoll().getValue();
        inputs.headingRateDPS = m_gyro.getRate();
    }

    @Override
    public void resetHeading() {
        m_gyro.setYaw(0.0);
    }

    @Override
    public void setHeading(Rotation2d heading) {
        m_gyro.setYaw(heading.getDegrees());
    }
}
