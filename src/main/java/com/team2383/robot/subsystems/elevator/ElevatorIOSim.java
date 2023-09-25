package com.team2383.robot.subsystems.elevator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class ElevatorIOSim implements ElevatorIO {
    private final LinearSystem<N1, N1, N1> m_motor;
    private Matrix<N1, N1> m_X = new Matrix<>(N1.instance, N1.instance);

    private double volts;
    private double extension = 0;

    public ElevatorIOSim() {
        m_motor = LinearSystemId.identifyVelocitySystem(ElevatorConstants.kV, ElevatorConstants.kA);
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        m_X = m_motor.calculateX(m_X, VecBuilder.fill(volts), 0.02);

        extension += m_X.get(0, 0) * 0.02;

        inputs.velocityMPS = m_X.get(0, 0);

        inputs.positionM = extension;
        inputs.appliedVolts = volts;
    }

    public void setVoltage(double voltage) {
        volts = voltage;
    }

    public void forcePosition(double positionMeters) {
        extension = positionMeters;
    }
}
