package com.team2383.robot.subsystems.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIONeo implements PivotIO {
    private final CANSparkMax m_pivotMotor;

    public PivotIONeo() {
        m_pivotMotor = new CANSparkMax(PivotConstants.kPivotMotorID, MotorType.kBrushless);
        m_pivotMotor.getEncoder().setPositionConversionFactor(0);
        m_pivotMotor.getEncoder().setVelocityConversionFactor(0);

    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotAngle = MathUtil.angleModulus(m_pivotMotor.getEncoder().getPosition());
        inputs.velocityRadPerS = m_pivotMotor.getEncoder().getVelocity();
        inputs.current = m_pivotMotor.getOutputCurrent();
        inputs.appliedVolts = m_pivotMotor.getAppliedOutput() * 12;
    }

    public void setVoltage(double voltage) {
        m_pivotMotor.setVoltage(voltage);
    }

    public void forceAngle(Rotation2d angle) {
    }

}
