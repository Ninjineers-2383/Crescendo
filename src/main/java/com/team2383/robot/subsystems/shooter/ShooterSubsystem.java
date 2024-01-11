package com.team2383.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO shooter;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public ShooterSubsystem(ShooterIO io) {
        shooter = io;
    }

    @Override
    public void periodic() {
        shooter.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setPower(double power) {
        shooter.setPower(power);
    }
}
