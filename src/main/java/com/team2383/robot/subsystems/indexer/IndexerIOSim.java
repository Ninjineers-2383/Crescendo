package com.team2383.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {
    private final FlywheelSim m_leftSim;
    private final FlywheelSim m_rightSim;

    private double m_leftPower = 0.0;
    private double m_rightPower = 0.0;

    public IndexerIOSim() {
        m_leftSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1.5E-5);
        m_rightSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1.5E-5);

    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.leftPower = m_leftPower;
        inputs.rightPower = m_rightPower;
    }

    @Override
    public void setLeftPower(double power) {
        m_leftSim.setInputVoltage(power * 12);
    }

    @Override
    public void setRightPower(double power) {
        m_rightSim.setInputVoltage(power * 12);
    }
}
