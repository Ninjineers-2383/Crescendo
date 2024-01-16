package com.team2383.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim m_leftFlywheelSim;
    private final FlywheelSim m_rightFlywheelSim;

    private double m_power = 0.0;

    public ShooterIOSim() {
        m_leftFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1.5E-5);
        m_rightFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1.5E-5);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftVoltage = m_power;
        inputs.leftCurrent = m_leftFlywheelSim.getCurrentDrawAmps();
        inputs.rightCurrent = m_rightFlywheelSim.getCurrentDrawAmps();
    }

    @Override
    public void setRPM(double RPM) {
        m_leftFlywheelSim.setInputVoltage(RPM);
        m_rightFlywheelSim.setInputVoltage(RPM);

        m_power = RPM;
    }
}
