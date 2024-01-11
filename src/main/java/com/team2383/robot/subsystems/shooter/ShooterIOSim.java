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
        inputs.power = m_power;
        inputs.leftCurrent = m_leftFlywheelSim.getCurrentDrawAmps();
        inputs.rightCurrent = m_rightFlywheelSim.getCurrentDrawAmps();
    }

    @Override
    public void setPower(double power) {
        m_leftFlywheelSim.setInputVoltage(power * 12);
        m_rightFlywheelSim.setInputVoltage(power * 12);

        m_power = power;
    }
}
