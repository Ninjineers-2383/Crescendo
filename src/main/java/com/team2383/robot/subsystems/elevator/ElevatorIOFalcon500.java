package com.team2383.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOFalcon500 implements ElevatorIO {

    private final TalonFX m_elevatorLeft;
    private final TalonFX m_elevatorRight;

    public ElevatorIOFalcon500(String CANbus) {
        m_elevatorLeft = new TalonFX(ElevatorConstants.kElevatorLeftID, CANbus);
        m_elevatorRight = new TalonFX(ElevatorConstants.kElevatorRightID, CANbus);
        m_elevatorLeft.setPosition(0);
        m_elevatorRight.setPosition(0);
        m_elevatorLeft.setInverted(false);
        m_elevatorRight.setInverted(true);

    }

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.velocityMPS = m_elevatorLeft.getVelocity().getValue() * 0.0665;

        inputs.positionM = m_elevatorLeft.getRotorPosition().getValue() * 0.0665;

        inputs.appliedVolts = m_elevatorLeft.getDutyCycle().getValue() * m_elevatorLeft.getSupplyVoltage().getValue();

        inputs.current = m_elevatorLeft.getSupplyCurrent().getValue();
    }

    public void setVoltage(double voltage) {
        m_elevatorLeft.setVoltage(voltage);
        m_elevatorRight.setVoltage(-voltage);
    }

    public void forcePosition(double positionMeters) {
        m_elevatorLeft.setPosition(positionMeters / 0.0665);
        m_elevatorRight.setPosition(positionMeters / 0.0665);

    }
}
