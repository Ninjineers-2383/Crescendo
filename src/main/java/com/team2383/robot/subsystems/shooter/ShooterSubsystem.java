package com.team2383.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.team2383.lib.util.mechanical_advantage.Alert;
import com.team2383.lib.util.mechanical_advantage.Alert.AlertType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO shooter;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Alert topMotorDisconnected;
    private final Alert bottomMotorDisconnected;
    private final Alert sideMotorDisconnected;

    public ShooterSubsystem(ShooterIO io) {
        shooter = io;

        topMotorDisconnected = new Alert("Top Shooter Motor Disconnected! CAN ID: " + ShooterConstants.kTopMotorID,
                AlertType.WARNING);

        bottomMotorDisconnected = new Alert(
                "Bottom Shooter Motor Disconnected! CAN ID: " + ShooterConstants.kBottomMotorID,
                AlertType.WARNING);

        sideMotorDisconnected = new Alert(
                "Spinner Motor Disconnected! CAN ID: " + ShooterConstants.kSideMotorID,
                AlertType.WARNING);
    }

    @Override
    public void periodic() {
        shooter.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        topMotorDisconnected.set(!inputs.topMotorConnected);
        bottomMotorDisconnected.set(!inputs.bottomMotorConnected);
        sideMotorDisconnected.set(!inputs.sideMotorConnected);
    }

    public void setTopBottomRPM(double RPM, double differential) {
        shooter.setTopBottomRPM(RPM, differential);
    }

    public void setTopBottomRPM(double RPM) {
        setTopBottomRPM(RPM, 0);
    }

    public void setSideRPM(double RPM) {
        shooter.setSideRPM(RPM);
    }

    public double getTopBottomRPM() {
        return inputs.topBottomSetpointRPM;
    }

    public void setTopBottomVoltage(double voltage) {
        shooter.setTopBottomVoltage(voltage);
    }

    public void setSideVoltage(double voltage) {
        shooter.setSideVoltage(voltage);
    }

    public double[] getVoltages() {
        return new double[] { inputs.topVoltage, inputs.bottomVoltage, inputs.sideVoltage };
    }

    public double[] getPositions() {
        return new double[] { inputs.topPosition, inputs.bottomPosition, inputs.sidePosition };
    }

    public double[] getVelocities() {
        return new double[] { inputs.topVelocity, inputs.bottomVelocity, inputs.sideVelocity };
    }
}
