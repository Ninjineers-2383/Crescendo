package com.team2383.robot.subsystems.trapArm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapArmSubsystem extends SubsystemBase {
    private final TrapArmIO io;
    private final TrapArmIOInputsAutoLogged inputs = new TrapArmIOInputsAutoLogged();

    public TrapArmSubsystem(TrapArmIO trapArmIO) {
        io = trapArmIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("TrapArm", inputs);
    }

    public void setAngle(double angle) {
        io.setAngle(angle);
    }

    public void setVelocity(double velocity) {
        io.setAngle(inputs.pivotAngle + 0.02 * velocity);
    }

}
