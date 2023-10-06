package com.team2383.robot.subsystems.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {
    private final DCMotor m_motorModel = DCMotor.getNEO(1);
    private final SingleJointedArmSim m_sim = new SingleJointedArmSim(m_motorModel,
            WristConstants.kWristMotorGearRatio, 1, 1, Math.toRadians(0),
            Math.toRadians(60), true);

    private double volts;

    /** Updates the set of loggable inputs. */
    public void updateInputs(WristIOInputs inputs) {
        m_sim.update(0.02);
        inputs.wristAngle = m_sim.getAngleRads();
        inputs.velocityRadPerSec = m_sim.getVelocityRadPerSec();
        inputs.appliedVolts = volts;
    }

    public void setVoltage(double voltage) {
        m_sim.setInputVoltage(voltage);
        volts = voltage;
    }

}
