package com.team2383.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.team2383.robot.Constants;
import com.team2383.robot.subsystems.orchestra.OrchestraContainer;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class PivotIOFalcon implements PivotIO {
    private final CANcoder encoder = new CANcoder(PivotConstants.kEncoderID, Constants.kCANivoreBus);

    private final TalonFX leftMotor = new TalonFX(PivotConstants.kLeftMotorID, Constants.kCANivoreBus);
    private final TalonFX rightMotor = new TalonFX(PivotConstants.kRightMotorID, Constants.kCANivoreBus);

    private final MotionMagicExpoTorqueCurrentFOC motionMagicOut = new MotionMagicExpoTorqueCurrentFOC(0);
    private final MotionMagicConfigs motionMagicConfigs;

    private final Follower follower = new Follower(PivotConstants.kLeftMotorID, false);

    private Slot0Configs leftConfigs = new Slot0Configs();
    private Slot0Configs rightConfigs = new Slot0Configs();

    public PivotIOFalcon() {
        OrchestraContainer.getInstance().addMotor(leftMotor);
        OrchestraContainer.getInstance().addMotor(rightMotor);

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Use max possible velocity

        motionMagicConfigs.MotionMagicExpo_kV = 0; // TODO
        motionMagicConfigs.MotionMagicExpo_kA = 0; // TODO

        setPIDController(PivotConstants.kPIDController);
        setFeedforward(PivotConstants.kFeedforwardController);

        leftMotor.getConfigurator().apply(motionMagicConfigs);
        rightMotor.getConfigurator().apply(motionMagicConfigs);

        rightMotor.setControl(follower);

        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.FeedbackRemoteSensorID = PivotConstants.kEncoderID;
        feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedback.SensorToMechanismRatio = 1;
        feedback.FeedbackRotorOffset = -PivotConstants.kEncoderOffset;

        leftMotor.getConfigurator().apply(feedback);
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.current = leftMotor.getTorqueCurrent().getValueAsDouble();
        inputs.appliedVolts = leftMotor.getDutyCycle().getValue() * rightMotor.getSupplyVoltage().getValue();
        inputs.pivotAngle = leftMotor.getPosition().getValue() ;
        inputs.velocityRadPerS = encoder.getVelocity().getValue() * 2 * Math.PI;
        inputs.desiredAngle = motionMagicOut.Position;
    }

    @Override
    public void setAngle(double angle) {
        motionMagicOut.withPosition(angle);
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

        motionMagicConfigs.MotionMagicExpo_kV = feedforward.kv;
        motionMagicConfigs.MotionMagicExpo_kA = feedforward.ka;

        leftMotor.getConfigurator().apply(motionMagicConfigs);
    }

}
