package com.team2383.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import com.team2383.lib.util.mechanical_advantage.Alert;
import com.team2383.lib.util.mechanical_advantage.Alert.AlertType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final FeederIO feeder;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    private final Alert feederDisconnected = new Alert(
            "Feeder Motor Disconnected! CAN ID: " + FeederConstants.kRearMotorID, AlertType.WARNING);;

    public FeederSubsystem(FeederIO io) {
        feeder = io;
    }

    @Override
    public void periodic() {
        feeder.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        feederDisconnected.set(!inputs.motorConnected);

    }

    public void setPower(double power) {
        feeder.setPower(power);
    }

    public double getPower() {
        return inputs.power;
    }
}
