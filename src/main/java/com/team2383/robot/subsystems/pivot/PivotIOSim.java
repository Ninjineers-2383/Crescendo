package com.team2383.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
    private final DCMotor m_motorModel = DCMotor.getNEO(2);
    private final SingleJointedArmSim m_sim = new SingleJointedArmSim(m_motorModel,
            PivotConstants.kPivotMotorGearRatio, 1, 1, Math.toRadians(0),
            Math.toRadians(60), true, 0);

    private double volts;

    /** Updates the set of loggable inputs. */
    public void updateInputs(PivotIOInputs inputs) {
        m_sim.update(0.02);
        inputs.pivotAngle = m_sim.getAngleRads();
        inputs.velocityRadPerS = m_sim.getVelocityRadPerSec();
        inputs.appliedVolts = volts;
    }

    public void setVoltage(double voltage) {
        m_sim.setInputVoltage(voltage);
        volts = voltage;
    }

}
