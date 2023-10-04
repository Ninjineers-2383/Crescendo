package com.team2383.robot.subsystems.elevator;

import com.ctre.phoenixpro.hardware.TalonFX;
import com.team2383.lib.math.Conversions;

public class ElevatorIOFalcon500 implements ElevatorIO {

    private final TalonFX m_elevatorLeft;
    private final TalonFX m_elevatorRight;
 
    public ElevatorIOFalcon500(String CANbus) {
        m_elevatorLeft = new TalonFX(ElevatorConstants.kElevatorLeftID, CANbus);
        m_elevatorRight = new TalonFX(ElevatorConstants.kElevatorRightID, CANbus);
        m_elevatorLeft.setInverted(false);
        m_elevatorRight.setInverted(true);

    }

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.velocityMPS = Conversions.RPSToMPS(
                m_elevatorLeft.getVelocity().getValue(),
                0,
                ElevatorConstants.kElevatorGearRatio);

        inputs.positionM = Conversions.rotationsToMeters(
                m_elevatorLeft.getPosition().getValue(),
                0,
                ElevatorConstants.kElevatorGearRatio);
                
        inputs.appliedVolts = m_elevatorLeft.getDutyCycle().getValue() * m_elevatorLeft.getSupplyVoltage().getValue();

        inputs.current = m_elevatorLeft.getStatorCurrent().getValue();
    }

    public void setVoltage(double voltage) {
        m_elevatorLeft.setVoltage(voltage);
        m_elevatorRight.setVoltage(voltage);
    }

    public void forcePosition(double positionMeters) {
        m_elevatorLeft.setRotorPosition(Conversions.metersToRotations(
            positionMeters, 0, ElevatorConstants.kElevatorGearRatio));
        m_elevatorRight.setRotorPosition(Conversions.metersToRotations(
            positionMeters, 0, ElevatorConstants.kElevatorGearRatio));

    }
}
