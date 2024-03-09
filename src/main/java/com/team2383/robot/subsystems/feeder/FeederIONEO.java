package com.team2383.robot.subsystems.feeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class FeederIONEO implements FeederIO {
    private final CANSparkMax feeder;

    private final DigitalInput m_beamBreak;

    private double voltage = 0;

    public FeederIONEO(int id) {
        feeder = new CANSparkMax(id, MotorType.kBrushless);

        m_beamBreak = new DigitalInput(1);

    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.motorConnected = feeder.getLastError() == REVLibError.kOk;
        inputs.current = feeder.getOutputCurrent();
        inputs.power = voltage;

        inputs.beamBreakTripped = !m_beamBreak.get();
    }

    @Override
    public void setPower(double power) {
        voltage = power * 12.0;
        feeder.setVoltage(voltage);

    }
}
