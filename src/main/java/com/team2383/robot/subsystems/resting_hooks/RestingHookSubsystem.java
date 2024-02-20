package com.team2383.robot.subsystems.resting_hooks;

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

    /**
     * This power must be positive. The hooks are on a ratchet
     * 
     * @param power
     *            the power to set the hooks to [0, 1]
     */
    public void setPower(double power) {
        if (power < 0.0) {
            throw new IllegalArgumentException("RestingHook power must be positive");
        }
        io.setPower(power);
    }

    /**
     * This is a little dangerous because it may slam into the bolts under
     * the hooks and push very hard. It is also the only reliable method as we
     * don't have encoders.
     */
    public boolean isDone() {
        return inputs.current > 10.0;
    }

}
