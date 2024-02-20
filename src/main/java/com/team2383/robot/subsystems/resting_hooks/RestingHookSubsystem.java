package com.team2383.robot.subsystems.restingHooks;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RestingHookSubsystem extends SubsystemBase {
    private final RestingHookIO io;
    private final RestingHookIOInputsAutoLogged inputs = new RestingHookIOInputsAutoLogged();

    public RestingHookSubsystem(RestingHookIO restingHookIO) {
        io = restingHookIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("RestingHook", inputs);
    }

    public void setPower(double power) {
        io.setPower(power);
    }

}
