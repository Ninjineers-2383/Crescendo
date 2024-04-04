package com.team2383.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim m_topFlywheelSim;
    private final FlywheelSim m_bottomFlywheelSim;

    private final FlywheelSim m_sideFlywheelSim;

    private final PIDController m_topBottomPIDController = new PIDController(1, 0, 0);

    private final PIDController m_sidePIDController = new PIDController(1, 0, 0);

    private double m_topVoltage = 0.0;
    private double m_bottomVoltage = 0.0;

    private double m_setTopBottomRPM = 0.0;
    private double m_setSideRPM = 0.0;

    public ShooterIOSim() {
        m_topFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1.5E-5);
        m_bottomFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1.5E-5);

        m_sideFlywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, 1.5E-5);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topVoltage = m_topVoltage;
        inputs.bottomVoltage = m_bottomVoltage;

        inputs.topCurrent = m_topFlywheelSim.getCurrentDrawAmps();
        inputs.bottomCurrent = m_bottomFlywheelSim.getCurrentDrawAmps();

        inputs.topVelocity = m_setTopBottomRPM;
        inputs.bottomVelocity = m_setTopBottomRPM;

        inputs.topSetpoint = m_setTopBottomRPM;
        inputs.bottomSetpoint = m_setTopBottomRPM;
        inputs.sideSetpoint = m_setSideRPM;

        setTopBottomVoltage(
                m_topBottomPIDController.calculate(m_topFlywheelSim.getAngularVelocityRPM(), m_setTopBottomRPM));

        setSideVoltage(m_sidePIDController.calculate(m_sideFlywheelSim.getAngularVelocityRPM(), m_setSideRPM));

        m_topFlywheelSim.update(0.02);
        m_bottomFlywheelSim.update(0.02);
    }

    @Override
    public void setTopBottomRPM(double RPM, double differential) {
        m_topBottomPIDController.setSetpoint(RPM);

        m_setTopBottomRPM = RPM;
    }

    @Override
    public void setSideRPM(double RPM) {
        m_sidePIDController.setSetpoint(RPM);

        m_setSideRPM = RPM;
    }

    @Override
    public void setTopBottomVoltage(double voltage) {
        m_topFlywheelSim.setInputVoltage(voltage);
        m_bottomFlywheelSim.setInputVoltage(voltage);
    }
}
