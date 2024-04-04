package com.team2383.robot.subsystems.drivetrain.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIONavX implements GyroIO {
    private final AHRS m_gyro;

    public GyroIONavX() {
        this.m_gyro = new AHRS();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true; // TODO: Figure out how to get connected status
        inputs.headingDeg = -m_gyro.getAngle();
        inputs.rollDeg = m_gyro.getRoll();
        inputs.headingRateDPS = m_gyro.getRate();
        inputs.headingAdjustment = m_gyro.getAngleAdjustment();
    }

    @Override
    public void resetHeading() {
        // m_gyro.setAngleAdjustment(-m_gyro.getAngle());
    }

    @Override
    public void setHeading(Rotation2d heading) {
        m_gyro.setAngleAdjustment(0);
        m_gyro.setAngleAdjustment((-heading.getDegrees()) - m_gyro.getAngle());
    }
}