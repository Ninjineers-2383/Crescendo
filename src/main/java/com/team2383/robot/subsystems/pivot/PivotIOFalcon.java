package com.team2383.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2383.robot.Constants;

public class PivotIOFalcon implements PivotIO {
    private final CANcoder encoder = new CANcoder(PivotConstants.kEncoderID, Constants.kCANivoreBus);

    private final TalonFX leftMotor = new TalonFX(PivotConstants.kLeftMotorID, Constants.kCANivoreBus);
    private final TalonFX rightMotor = new TalonFX(PivotConstants.kRightMotorID, Constants.kCANivoreBus);

    private final MotionMagicExpoTorqueCurrentFOC motionMagicOut = new MotionMagicExpoTorqueCurrentFOC(0);
    private final MotionMagicConfigs motionMagicConfigs;

    private final VoltageOut voltageOut = new VoltageOut(0, true, true, false, false);

    private final Follower follower = new Follower(PivotConstants.kLeftMotorID, false);

    public PivotIOFalcon() {
        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Use max possible velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0; // TODO
        motionMagicConfigs.MotionMagicExpo_kA = 0; // TODO

        rightMotor.setControl(follower);
    }

    @Override
    public void setVoltage(double voltage) {
        voltageOut.withOutput(voltage);

        leftMotor.setControl(voltageOut);
    }

}
