package com.team2383.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {
    private final FlywheelSim m_sim;

    private double m_power = 0.0;

    public IndexerIOSim() {
        m_sim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1.5E-5);

    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.power = m_power;
        m_sim.setInputVoltage(m_power * 12);

        m_sim.update(0.02);
    }

    @Override
    public void setPower(double power) {
        m_power = power;
    }
}
