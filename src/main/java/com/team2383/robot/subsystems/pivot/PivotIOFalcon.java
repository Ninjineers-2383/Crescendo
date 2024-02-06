package com.team2383.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.team2383.robot.Constants;
import com.team2383.robot.subsystems.orchestra.OrchestraContainer;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class PivotIOFalcon implements PivotIO {
    private final CANcoder encoder = new CANcoder(PivotConstants.kEncoderID, Constants.kCANivoreBus);

    private final TalonFX leftMotor = new TalonFX(PivotConstants.kLeftMotorID, Constants.kCANivoreBus);
    private final TalonFX rightMotor = new TalonFX(PivotConstants.kRightMotorID, Constants.kCANivoreBus);

    private final MotionMagicVoltage motionMagicOut = new MotionMagicVoltage(0);
    private final MotionMagicConfigs motionMagicConfigs;

    private final Follower follower = new Follower(PivotConstants.kLeftMotorID, false);

    private Slot0Configs leftConfigs = new Slot0Configs();
    private Slot0Configs rightConfigs = new Slot0Configs();

    private double positionOffset = 0.0;

    public PivotIOFalcon() {
        OrchestraContainer.getInstance().addMotor(leftMotor);
        OrchestraContainer.getInstance().addMotor(rightMotor);

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Use max possible velocity

        motionMagicConfigs.MotionMagicExpo_kV = PivotConstants.kV;
        motionMagicConfigs.MotionMagicExpo_kA = PivotConstants.kA;
        motionMagicConfigs.MotionMagicAcceleration = 0.25;
        motionMagicConfigs.MotionMagicCruiseVelocity = 3;

        setPIDController(PivotConstants.kPIDController);
        setFeedforward(PivotConstants.kFeedforwardController);

        leftMotor.getConfigurator().apply(motionMagicConfigs);
        rightMotor.getConfigurator().apply(motionMagicConfigs);

        rightMotor.setControl(follower);

        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.FeedbackRemoteSensorID = PivotConstants.kEncoderID;
        feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedback.SensorToMechanismRatio = 1;
        feedback.RotorToSensorRatio = PivotConstants.kPivotMotorGearRatio;

        leftMotor.getConfigurator().apply(feedback);

        positionOffset = encoder.getAbsolutePosition().getValueAsDouble() - PivotConstants.kEncoderOffset;
        leftMotor.setPosition(0);
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.current = leftMotor.getTorqueCurrent().getValue();
        inputs.appliedVolts = leftMotor.getDutyCycle().getValue() * rightMotor.getSupplyVoltage().getValue();
        inputs.velocityRadPerS = encoder.getVelocity().getValue() * 2 * Math.PI;

        inputs.desiredAcceleration = (leftMotor.getClosedLoopReferenceSlope().getValue()
                - inputs.desiredVelocity) / 2.0;
        inputs.currentAcceleration = leftMotor.getAcceleration().getValue();

        inputs.desiredVelocity = leftMotor.getClosedLoopReferenceSlope().getValue();
        inputs.currentVelocity = leftMotor.getVelocity().getValue();

        inputs.pivotAngle = leftMotor.getPosition().getValue() - positionOffset;
        inputs.desiredAngle = motionMagicOut.Position * 2 * Math.PI - positionOffset;
        inputs.currentDesired = leftMotor.getClosedLoopReference().getValue() - positionOffset;
    }

    @Override
    public void setAngle(double angle) {
        motionMagicOut.withPosition(Units.radiansToRotations(angle) - positionOffset);
        leftMotor.setControl(motionMagicOut);
    }

    @Override
    public void setPIDController(PIDController controller) {
        leftConfigs.kP = controller.getP();
        leftConfigs.kI = controller.getI();
        leftConfigs.kD = controller.getD();

        leftMotor.getConfigurator().apply(leftConfigs);

        rightConfigs.kP = controller.getP();
        rightConfigs.kI = controller.getI();
        rightConfigs.kD = controller.getD();

        rightMotor.getConfigurator().apply(rightConfigs);
    }

    @Override
    public void setFeedforward(ArmFeedforward feedforward) {
        leftConfigs.kA = feedforward.ka;
        leftConfigs.kG = feedforward.kg;
        leftConfigs.kS = feedforward.ks;
        leftConfigs.kV = feedforward.kv;

        leftConfigs.GravityType = GravityTypeValue.Arm_Cosine;

        leftMotor.getConfigurator().apply(leftConfigs);

        rightConfigs.kA = feedforward.ka;
        rightConfigs.kG = feedforward.kg;
        rightConfigs.kS = feedforward.ks;
        rightConfigs.kV = feedforward.kv;

        rightConfigs.GravityType = GravityTypeValue.Arm_Cosine;

        rightMotor.getConfigurator().apply(rightConfigs);
    }

}
