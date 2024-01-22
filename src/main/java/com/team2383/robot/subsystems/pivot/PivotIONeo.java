package com.team2383.robot.subsystems.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIONeo implements PivotIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    public PivotIONeo() {
        leftMotor = new CANSparkMax(PivotConstants.kLeftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(PivotConstants.kRightMotorID, MotorType.kBrushless);
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.current = leftMotor.getOutputCurrent();
        inputs.appliedVolts = leftMotor.getAppliedOutput() * 12;
    }

    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    public void forceAngle(Rotation2d angle) {
    }

}
