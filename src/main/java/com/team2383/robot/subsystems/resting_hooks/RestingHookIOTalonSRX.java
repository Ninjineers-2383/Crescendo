package com.team2383.robot.subsystems.restingHooks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RestingHookIOTalonSRX implements RestingHookIO {
    private final TalonSRX leftHook;
    private final TalonSRX rightHook;

    public RestingHookIOTalonSRX() {
        leftHook = new TalonSRX(RestingHookConstants.kLeftHookID);
        rightHook = new TalonSRX(RestingHookConstants.kRightHookID);

        rightHook.follow(leftHook);
    }

    @Override
    public void updateInputs(RestingHookIOInputs inputs) {
        inputs.voltage = leftHook.getMotorOutputVoltage();
        inputs.current = leftHook.getSupplyCurrent();
    }

    @Override
    public void setPower(double power) {
        leftHook.set(ControlMode.PercentOutput, power);
    }
}
