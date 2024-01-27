package com.team2383.robot.subsystems.pivot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIONeo implements PivotIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final CANcoder encoder;

    public PivotIONeo() {
        leftMotor = new CANSparkMax(PivotConstants.kLeftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(PivotConstants.kRightMotorID, MotorType.kBrushless);
        encoder = new CANcoder(PivotConstants.kEncoderID, "Drive");

        leftMotor.setInverted(false);
        rightMotor.setInverted(false);
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.current = leftMotor.getOutputCurrent();
        inputs.appliedVolts = leftMotor.getAppliedOutput() * 12;
        inputs.pivotAngle = encoder.getPosition().getValue() * 2 * Math.PI - PivotConstants.kEncoderOffset;
        inputs.velocityRadPerS = encoder.getVelocity().getValue() * 2 * Math.PI;
    }

    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);

    }

    public void forceAngle(Rotation2d angle) {
    }

}
