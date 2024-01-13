package com.team2383.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FeederIOSim implements FeederIO {
    private final FlywheelSim m_feederSim;

    private double m_power = 0.0;

    public FeederIOSim() {
        m_feederSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1.5E-5);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.power = m_power;
        // inputs.current = m_feederSim.getCurrentDrawAmps();
    }

    @Override
    public void setPower(double power) {
        m_feederSim.setInputVoltage(power * 12);
        m_power = power;
    }
}
