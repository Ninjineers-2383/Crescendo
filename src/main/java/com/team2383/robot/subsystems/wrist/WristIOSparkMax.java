package com.team2383.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.lib.math.AngularVelocityWrapper;
import com.team2383.lib.swerve.AbsoluteMagEncoder;
import com.team2383.lib.swerve.IAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class WristIOSparkMax implements WristIO {

    private final CANSparkMax m_wristMotor;

    private final IAbsoluteEncoder m_absEncoder;

    public WristIOSparkMax() {
        m_wristMotor = new CANSparkMax(WristConstants.kWristMotorID, MotorType.kBrushless);

        m_wristMotor.restoreFactoryDefaults();
        m_wristMotor.setSmartCurrentLimit(WristConstants.kMaxCurrent);

        m_wristMotor.setInverted(true);
        m_wristMotor.setIdleMode(IdleMode.kBrake);
        m_wristMotor.getEncoder().setPositionConversionFactor(1);
        m_wristMotor.getEncoder().setVelocityConversionFactor(1);

        m_absEncoder = new AbsoluteMagEncoder(WristConstants.kAbsEncoderID);
        Timer.delay(2);
        forceAngle(
                Rotation2d.fromRadians(m_absEncoder.getAbsoluteAngle().getRadians() - WristConstants.kEncoderOffset));
    }

    public void updateInputs(WristIOInputs inputs) {
        inputs.wristAngle = m_absEncoder.getAbsoluteAngle().getRadians() - WristConstants.kEncoderOffset;
        inputs.velocityRadPerSec = m_wristMotor.getEncoder().getVelocity() * ((2 * Math.PI) / 60.0)
                / WristConstants.kWristMotorGearRatio;
        inputs.appliedVolts = m_wristMotor.getAppliedOutput() * m_wristMotor.getBusVoltage();
        inputs.current = m_wristMotor.getOutputCurrent();
        inputs.absoluteWristAngle = m_absEncoder.getAbsoluteAngle().getRadians();
        inputs.absoluteWristAngleOffset = m_absEncoder.getAbsoluteAngle().getRadians() - WristConstants.kEncoderOffset;

    }

    public void setVoltage(double voltage) {
        m_wristMotor.set(voltage / m_wristMotor.getBusVoltage());
    }

    public void forceAngle(Rotation2d angle) {
        m_wristMotor.getEncoder().setPosition(angle.getRotations() * WristConstants.kWristMotorGearRatio);
    }

}
