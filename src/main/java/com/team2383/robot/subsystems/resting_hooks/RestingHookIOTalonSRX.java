package com.team2383.robot.subsystems.restingHooks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RestingHookIOTalonSRX implements RestingHookIO {
    private final TalonSRX leftHook;
    private final TalonSRX rightHook;

    double voltage = 0;

    public RestingHookIOTalonSRX() {
        leftHook = new TalonSRX(RestingHookConstants.kLeftHookID);
        rightHook = new TalonSRX(RestingHookConstants.kRightHookID);

        rightHook.follow(leftHook);
    }

    @Override
    public void updateInputs(RestingHookIOInputs inputs) {
    }

    @Override
    public void setPower(double power) {
        voltage = power * 12.0;
        leftHook.set(ControlMode.PercentOutput, power);
    }
}
