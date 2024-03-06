package com.team2383.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOPigeon implements GyroIO {
    private final Pigeon2 m_gyro;

    private final StatusSignal<Double> getYaw;
    private final StatusSignal<Double> getRoll;

    public GyroIOPigeon(int id, String CANbus) {
        this.m_gyro = new Pigeon2(id, CANbus);

        getYaw = m_gyro.getYaw();
        getRoll = m_gyro.getRoll();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(getYaw, getRoll).isOK();
        inputs.headingDeg = getYaw.getValueAsDouble();
        inputs.rollDeg = getRoll.getValueAsDouble();
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
