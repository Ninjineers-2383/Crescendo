package com.team2383.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotor m_motorModel = DCMotor.getFalcon500(2);
    private final ElevatorSim m_sim = new ElevatorSim(m_motorModel, 7.75, Units.lbsToKilograms(3),
            Units.inchesToMeters(2), 0, Units.inchesToMeters(45), true, null);

    private double volts;
    private double extension = 0;

    public ElevatorIOSim() {
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        m_sim.update(0.02);

        extension = m_sim.getPositionMeters();

        inputs.velocityMPS = m_sim.getVelocityMetersPerSecond();

        inputs.positionM = extension;
        inputs.appliedVolts = volts;
    }

    public void setVoltage(double voltage) {
        m_sim.setInputVoltage(voltage);
        volts = voltage;
    }

    public void forcePosition(double positionMeters) {
        extension = positionMeters;
    }
}
