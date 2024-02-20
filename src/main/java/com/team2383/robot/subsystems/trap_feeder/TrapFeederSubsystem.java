package com.team2383.robot.subsystems.trap_feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapFeederSubsystem extends SubsystemBase {
    private final TrapFeederIO io;
    private final TrapFeederIOInputsAutoLogged inputs = new TrapFeederIOInputsAutoLogged();

    public TrapFeederSubsystem(TrapFeederIO trapFeederIO) {
        io = trapFeederIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setPower(double power) {
        io.setPower(power);
    }

}
