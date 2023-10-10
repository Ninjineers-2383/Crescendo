package com.team2383.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final FeederIO feeder;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    public FeederSubsystem(FeederIO io) {
        feeder = io;
    }

    @Override
    public void periodic() {
        feeder.updateInputs(inputs);
        Logger.getInstance().processInputs("Feeder", inputs);
    }

    public void setPower(double power) {
        feeder.setPower(power);
    }

    public double getPower() {
        return inputs.power;
    }
}
