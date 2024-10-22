package com.team2383.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team2383.robot.Constants;
import com.team2383.robot.subsystems.orchestra.OrchestraContainer;
import com.team2383.robot.subsystems.pivot.PivotSubsystem.LashState;

import java.util.List;

public class PivotIOFalcon implements PivotIO {
    private final TalonFX leftMotorLeader;
    private final TalonFX rightMotorFollower;

    private final CANcoder encoder;

    private final StatusSignal<Double> internalPositionRotations;
    private final StatusSignal<Double> absolutePositionRotations;
    private final StatusSignal<Double> velocityRps;
    private final List<StatusSignal<Double>> appliedVoltage;
    private final List<StatusSignal<Double>> supplyCurrent;
    private final List<StatusSignal<Double>> torqueCurrent;
    private final List<StatusSignal<Double>> tempCelsius;

    private final PositionVoltage positionOut = new PositionVoltage(0.0).withUpdateFreqHz(0.0);

    private final VoltageOut voltage = new VoltageOut(0.0).withUpdateFreqHz(0.0);

    private SlotConfigs forwardConfigs;
    private SlotConfigs backConfigs;

    private double offset = 0;

    private boolean offsetSet = false;

    private double kSpring = PivotConstants.kGains.kSpring();

    private LashState lastStatePrev;

    public PivotIOFalcon() {
        leftMotorLeader = new TalonFX(PivotConstants.kLeftMotorID, Constants.kCANivoreBus);
        rightMotorFollower = new TalonFX(PivotConstants.kRightMotorID, Constants.kCANivoreBus);

        leftMotorLeader.setNeutralMode(NeutralModeValue.Coast);
        rightMotorFollower.setNeutralMode(NeutralModeValue.Coast);

        encoder = new CANcoder(PivotConstants.kEncoderID, Constants.kCANivoreBus);

        rightMotorFollower.setControl(new Follower(PivotConstants.kLeftMotorID, true));

        OrchestraContainer.getInstance().addMotor(leftMotorLeader);
        OrchestraContainer.getInstance().addMotor(rightMotorFollower);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.TorqueCurrent = new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(80)
                .withPeakReverseTorqueCurrent(-80);

        forwardConfigs = new SlotConfigs()
                .withKS(PivotConstants.kGains.kS())
                .withKV(PivotConstants.kGains.kV())
                .withKA(PivotConstants.kGains.kA())
                .withKG(PivotConstants.kGains.kG())
                .withGravityType(GravityTypeValue.Arm_Cosine);
        backConfigs = new SlotConfigs()
                .withKP(PivotConstants.kGains.kP())
                .withKI(PivotConstants.kGains.kI())
                .withKD(PivotConstants.kGains.kD())
                .withKG(PivotConstants.kGains.kG())
                .withGravityType(GravityTypeValue.Arm_Cosine);

        motorConfig.Slot0 = Slot0Configs.from(forwardConfigs);
        motorConfig.Slot1 = Slot1Configs.from(backConfigs);

        FeedbackConfigs feedback = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(encoder.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(1)
                .withRotorToSensorRatio(PivotConstants.kPivotMotorGearRatio);

        motorConfig.Feedback = feedback;

        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorLeader.getConfigurator().apply(motorConfig);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
                .withMagnetSensor(
                        new MagnetSensorConfigs()
                                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                                .withMagnetOffset(PivotConstants.kEncoderOffset)
                                .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

        encoder.getConfigurator().apply(encoderConfig);

        // Status signals
        internalPositionRotations = leftMotorLeader.getRotorPosition();
        absolutePositionRotations = leftMotorLeader.getPosition();
        velocityRps = leftMotorLeader.getVelocity();
        appliedVoltage = List.of(leftMotorLeader.getMotorVoltage(), rightMotorFollower.getMotorVoltage());
        supplyCurrent = List.of(leftMotorLeader.getSupplyCurrent(), rightMotorFollower.getSupplyCurrent());
        torqueCurrent = List.of(leftMotorLeader.getTorqueCurrent(), rightMotorFollower.getTorqueCurrent());
        tempCelsius = List.of(leftMotorLeader.getDeviceTemp(), rightMotorFollower.getDeviceTemp());

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                internalPositionRotations,
                absolutePositionRotations,
                velocityRps,
                appliedVoltage.get(0),
                appliedVoltage.get(1),
                supplyCurrent.get(0),
                supplyCurrent.get(1),
                torqueCurrent.get(0),
                torqueCurrent.get(1),
                tempCelsius.get(0),
                tempCelsius.get(1));

        // Optimize bus utilization
        rightMotorFollower.optimizeBusUtilization(1.0);
        leftMotorLeader.optimizeBusUtilization(1.0);
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.leftMotorConnected = BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRps,
                appliedVoltage.get(0),
                supplyCurrent.get(0),
                torqueCurrent.get(0),
                tempCelsius.get(0))
                .isOK();
        inputs.rightMotorConnected = BaseStatusSignal.refreshAll(
                appliedVoltage.get(1),
                supplyCurrent.get(1),
                torqueCurrent.get(1),
                tempCelsius.get(1))
                .isOK();

        inputs.encoderConnected = BaseStatusSignal.refreshAll(absolutePositionRotations).isOK();

        inputs.rotorPositionRot = internalPositionRotations.getValueAsDouble();
        inputs.absoluteEncoderPositionRot = absolutePositionRotations.getValueAsDouble() - offset;
        inputs.desiredPositionRot = leftMotorLeader.getClosedLoopReference().getValue() - offset;

        inputs.velocityRotPerSec = velocityRps.getValue();
        inputs.desiredVelocityRotPerSec = leftMotorLeader.getClosedLoopReferenceSlope().getValue();

        inputs.appliedVolts = appliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.supplyCurrentAmps = supplyCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.torqueCurrentAmps = torqueCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.tempCelcius = tempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();

        double position = leftMotorLeader.getPosition().getValueAsDouble();

        while (position - offset > 1.4 && !offsetSet) {
            offset += 1;
        }
        while (position - offset < -0.6 && !offsetSet) {
            offset -= 1;
        }

        offsetSet = true;
    }

    @Override
    public void setAngleRot(double angleRot, double velocityRotPerSec, PivotSubsystem.LashState lashState) {
        if (lashState != lastStatePrev) {
            leftMotorLeader.getConfigurator().apply(Slot1Configs.from(backConfigs));
        }

        leftMotorLeader.setControl(positionOut.withPosition(angleRot + offset).withVelocity(velocityRotPerSec)
                .withFeedForward(kSpring).withSlot(lashState == LashState.Forward ? 0 : 1));

        lastStatePrev = lashState;

    }

    @Override
    public void setVoltage(double volts) {
        leftMotorLeader.setControl(voltage.withOutput(volts));
    }

    @Override
    public void disable() {
        leftMotorLeader.disable();

        // ! Don't disable the follower, it will never turn back on unless you re-enable
        // ! it
        // rightMotorFollower.disable();
    }

    @Override
    public void setPIDController(double kP, double kI, double kD) {
        backConfigs.kP = kP;
        backConfigs.kI = kI;
        backConfigs.kD = kD;

        leftMotorLeader.getConfigurator().apply(Slot1Configs.from(backConfigs));
    }

    @Override
    public void setFeedforward(double kS, double kV, double kA, double kG, double kSpring) {
        forwardConfigs.kA = kA;
        forwardConfigs.kG = kG;
        backConfigs.kG = kG;
        forwardConfigs.kS = kS;
        forwardConfigs.kV = kV;

        this.kSpring = kSpring;

        forwardConfigs.GravityType = GravityTypeValue.Arm_Cosine;
        backConfigs.GravityType = GravityTypeValue.Arm_Cosine;

        leftMotorLeader.getConfigurator().apply(Slot0Configs.from(forwardConfigs));
        leftMotorLeader.getConfigurator().apply(Slot1Configs.from(backConfigs));

        System.out.println("Pivot Feedforward changes");
    }
}
