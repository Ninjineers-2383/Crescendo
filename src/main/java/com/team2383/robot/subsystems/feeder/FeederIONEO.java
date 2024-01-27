package com.team2383.robot.subsystems.feeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class FeederIONEO implements FeederIO {
    private final CANSparkMax leftMotor = new CANSparkMax(FeederConstants.kRightMotorID, MotorType.kBrushless);
    // private final CANSparkMax rightMotor = new
    // CANSparkMax(FeederConstants.kRightMotorID, MotorType.kBrushless);

    double voltage = 0;

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.leftCurrent = leftMotor.getOutputCurrent();
        // inputs.rightCurrent = rightMotor.getOutputCurrent();
        inputs.power = voltage;
    }

    @Override
    public void setPower(double power) {
        voltage = power * 12.0;
        leftMotor.setVoltage(voltage);
        // rightMotor.setVoltage(voltage);
    }
}
