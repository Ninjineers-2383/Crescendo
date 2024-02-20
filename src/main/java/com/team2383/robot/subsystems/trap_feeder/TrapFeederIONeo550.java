package com.team2383.robot.subsystems.trapFeeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class TrapFeederIONeo550 implements TrapFeederIO {
    private final CANSparkMax feeder;

    double voltage = 0;

    public TrapFeederIONeo550() {
        feeder = new CANSparkMax(TrapFeederConstants.kTrapFeederID, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(TrapFeederIOInputs inputs) {
        inputs.current = feeder.getOutputCurrent();
        inputs.power = voltage;
    }

    @Override
    public void setPower(double power) {
        voltage = power * 12.0;
        feeder.setVoltage(voltage);
    }
}
