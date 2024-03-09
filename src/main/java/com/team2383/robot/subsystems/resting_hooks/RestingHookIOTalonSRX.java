package com.team2383.robot.subsystems.resting_hooks;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RestingHookIOTalonSRX implements RestingHookIO {
    private final TalonSRX leftHook;
    private final TalonSRX rightHook;

    public RestingHookIOTalonSRX() {
        leftHook = new TalonSRX(RestingHookConstants.kLeftHookID);
        rightHook = new TalonSRX(RestingHookConstants.kRightHookID);

        leftHook.setInverted(true);
    }

    @Override
    public void updateInputs(RestingHookIOInputs inputs) {
        inputs.leftHookConnected = leftHook.getLastError() == ErrorCode.OK;
        inputs.rightHookConnected = rightHook.getLastError() == ErrorCode.OK;

        inputs.voltage = leftHook.getMotorOutputVoltage();
        inputs.current = leftHook.getSupplyCurrent();
    }

    @Override
    public void setPower(double power) {
        leftHook.set(ControlMode.PercentOutput, power);
        rightHook.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void setPowerSingle(double power) {
        leftHook.set(ControlMode.PercentOutput, power);
    }
}
