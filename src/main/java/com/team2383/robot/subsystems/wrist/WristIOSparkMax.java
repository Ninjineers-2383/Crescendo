package com.team2383.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.lib.math.AngularVelocityWrapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;


public class WristIOSparkMax implements WristIO{

    private final CANSparkMax m_wristMotor;

    private final DutyCycleEncoder m_absEncoder;

    private AngularVelocityWrapper m_velocity;
 
    public WristIOSparkMax(String CANbus) {
        m_wristMotor = new CANSparkMax(WristConstants.kWristMotorID, MotorType.kBrushless);

        m_wristMotor.restoreFactoryDefaults();
        m_wristMotor.setSmartCurrentLimit(WristConstants.kMaxCurrent);

        m_absEncoder = new DutyCycleEncoder(WristConstants.kAbsEncoderID);

        m_absEncoder.setPositionOffset(WristConstants.kEncoderOffset);

        m_velocity = new AngularVelocityWrapper(Rotation2d.fromRotations(m_absEncoder.get()));
    }

    public void updateInputs(WristIOInputs inputs) {
        inputs.wristAngle = m_absEncoder.get() * 2 * Math.PI;
        inputs.velocityRadPerSec = m_velocity.calculate(Rotation2d.fromRotations(inputs.wristAngle)).getRadians();
        inputs.appliedVolts = m_wristMotor.getAppliedOutput();
        inputs.current = m_wristMotor.getOutputCurrent();
    }

    public void setVoltage(double voltage) {
        m_wristMotor.setVoltage(voltage);
    }

    public void forceAngle(double angle) {

    }
    
}
