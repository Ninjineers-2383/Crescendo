package com.team2383.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterIONEO implements ShooterIO {
    private final CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.kLeftMotorID, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.kRightMotorID, MotorType.kBrushless);
    double voltage = 0;

    public ShooterIONEO() {
        leftMotor.getPIDController().setP(1);
        leftMotor.getPIDController().setI(0);
        leftMotor.getPIDController().setD(0);

        rightMotor.getPIDController().setP(1);
        rightMotor.getPIDController().setI(0);
        rightMotor.getPIDController().setD(0);

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftCurrent = leftMotor.getOutputCurrent();
        inputs.rightCurrent = rightMotor.getOutputCurrent();
        inputs.leftVoltage = voltage;
    }

    @Override
    public void setRPM(double RPM) {
        leftMotor.getPIDController().setReference(RPM, ControlType.kVelocity);
        rightMotor.getPIDController().setReference(RPM, ControlType.kVelocity);
    }
}
