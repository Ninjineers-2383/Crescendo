package com.team2383.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.team2383.robot.Constants;
import com.team2383.robot.subsystems.orchestra.OrchestraContainer;

public class PivotIOFalcon implements PivotIO {
    private final TalonFX leftMotorLeader;
    private final TalonFX rightMotorFollower;

    private final CANcoder encoder;

    private final PositionTorqueCurrentFOC positionOut = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    private final VoltageOut voltage = new VoltageOut(0.0).withUpdateFreqHz(0.0);

    private Slot0Configs configs;

    private double offset = 0;

    public PivotIOFalcon() {
        leftMotorLeader = new TalonFX(PivotConstants.kLeftMotorID, Constants.kCANivoreBus);
        rightMotorFollower = new TalonFX(PivotConstants.kRightMotorID, Constants.kCANivoreBus);

        encoder = new CANcoder(PivotConstants.kEncoderID, Constants.kCANivoreBus);

        rightMotorFollower.setControl(new Follower(PivotConstants.kLeftMotorID, true));

        OrchestraContainer.getInstance().addMotor(leftMotorLeader);
        OrchestraContainer.getInstance().addMotor(rightMotorFollower);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.TorqueCurrent = new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(80)
                .withPeakReverseTorqueCurrent(-80);

        configs = new Slot0Configs()
                .withKP(PivotConstants.kGains.kP())
                .withKI(PivotConstants.kGains.kI())
                .withKD(PivotConstants.kGains.kD())
                .withKS(PivotConstants.kGains.kS())
                .withKV(PivotConstants.kGains.kV())
                .withKA(PivotConstants.kGains.kA())
                .withKG(PivotConstants.kGains.kG())
                .withGravityType(GravityTypeValue.Arm_Cosine);

        motorConfig.Slot0 = configs;

        FeedbackConfigs feedback = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(encoder.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(1)
                .withRotorToSensorRatio(PivotConstants.kPivotMotorGearRatio);

        motorConfig.Feedback = feedback;

        leftMotorLeader.setInverted(true);
        leftMotorLeader.getConfigurator().apply(motorConfig);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
                .withMagnetSensor(
                        new MagnetSensorConfigs()
                                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                                .withMagnetOffset(PivotConstants.kEncoderOffset));

        encoder.getConfigurator().apply(encoderConfig);
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.current = leftMotorLeader.getTorqueCurrent().getValue();

        inputs.appliedVolts = leftMotorLeader.getDutyCycle().getValue()
                * rightMotorFollower.getSupplyVoltage().getValue();

        inputs.velocityRadPerS = encoder.getVelocity().getValue() * 2 * Math.PI;

        inputs.desiredVelocity = leftMotorLeader.getClosedLoopReferenceSlope().getValue();
        inputs.currentVelocity = leftMotorLeader.getVelocity().getValue();

        inputs.pivotAngle = leftMotorLeader.getPosition().getValue() - offset;
        inputs.currentDesiredAngle = leftMotorLeader.getClosedLoopReference().getValue() - offset;

        if (leftMotorLeader.getPosition().getValueAsDouble() > 0.9 && offset == 0) {
            offset = 1;
        }
    }

    @Override
    public void setAngleRadians(double angle) {
        leftMotorLeader.setControl(positionOut.withPosition(angle));
    }

    @Override
    public void setVoltage(double volts) {
        leftMotorLeader.setControl(voltage.withOutput(volts));
    }

    @Override
    public void disable() {
        leftMotorLeader.disable();
        rightMotorFollower.disable();
    }

    @Override
    public void setPIDController(double kP, double kI, double kD) {
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;

        leftMotorLeader.getConfigurator().apply(configs);
    }

    @Override
    public void setFeedforward(double kS, double kV, double kA, double kG) {
        configs.kA = kA;
        configs.kG = kG;
        configs.kS = kS;
        configs.kV = kV;

        configs.GravityType = GravityTypeValue.Arm_Cosine;

        leftMotorLeader.getConfigurator().apply(configs);
    }
}
