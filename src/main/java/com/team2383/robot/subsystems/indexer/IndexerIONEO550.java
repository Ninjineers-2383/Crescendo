package com.team2383.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;

public class IndexerIONEO550 implements IndexerIO {
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;

    public IndexerIONEO550() {
        m_leftMotor = new CANSparkMax(IndexerConstants.kLeftMotorID, CANSparkMax.MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(IndexerConstants.kRightMotorID, CANSparkMax.MotorType.kBrushless);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.leftPower = m_leftMotor.get();
        inputs.rightPower = m_rightMotor.get();
    }

    @Override
    public void setLeftPower(double power) {
        m_leftMotor.set(power);
    }

    @Override
    public void setRightPower(double power) {
        m_rightMotor.set(power);
    }
}
