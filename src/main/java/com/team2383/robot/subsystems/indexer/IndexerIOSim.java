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
    }

    @Override
    public void setPower(double power) {
        m_sim.setInputVoltage(power * 12);
    }
}
