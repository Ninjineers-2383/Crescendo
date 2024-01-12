package com.team2383.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterNEO implements ShooterIO {
    private final CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.kLeftMotorID, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.kRightMotorID, MotorType.kBrushless);

    double voltage = 0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftCurrent = leftMotor.getOutputCurrent();
        inputs.rightCurrent = rightMotor.getOutputCurrent();
        inputs.power = voltage;
    }

    @Override
    public void setPower(double power) {
        voltage = power * 12.0;
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }
}
