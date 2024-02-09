package com.team2383.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.team2383.robot.Constants;
import com.team2383.robot.subsystems.orchestra.OrchestraContainer;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class PivotIOFalconTrapezoidal implements PivotIO {
    private final CANcoder encoder = new CANcoder(PivotConstants.kEncoderID, Constants.kCANivoreBus);

    private final TalonFX leftMotor = new TalonFX(PivotConstants.kLeftMotorID, Constants.kCANivoreBus);
    private final TalonFX rightMotor = new TalonFX(PivotConstants.kRightMotorID, Constants.kCANivoreBus);

    private final Follower follower = new Follower(PivotConstants.kLeftMotorID, false);

    private final PositionVoltage positionOut = new PositionVoltage(0);

    private Slot0Configs leftConfigs = new Slot0Configs();
    private Slot0Configs rightConfigs = new Slot0Configs();

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            5, 7);

    private TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private double offset = 0;

    public PivotIOFalconTrapezoidal() {
        OrchestraContainer.getInstance().addMotor(leftMotor);
        OrchestraContainer.getInstance().addMotor(rightMotor);

        setPIDController(PivotConstants.kPIDController);
        setFeedforward(PivotConstants.kFeedforwardController);

        rightMotor.setControl(follower);

        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedback.SensorToMechanismRatio = 1;
        feedback.RotorToSensorRatio = PivotConstants.kPivotMotorGearRatio;

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderConfig.MagnetSensor.MagnetOffset = PivotConstants.kEncoderOffset;
        encoder.getConfigurator().apply(encoderConfig);

        leftMotor.getConfigurator().apply(feedback);

        // leftMotor.setPosition(0);

        goal = new TrapezoidProfile.State(leftMotor.getPosition().getValueAsDouble(), 0);
        setpoint = goal;
    }

    public void updateInputs(PivotIOInputs inputs) {
        setpoint = profile.calculate(0.02, setpoint, goal);

        leftMotor.setControl(positionOut.withPosition(setpoint.position).withVelocity(setpoint.velocity));

        inputs.current = leftMotor.getTorqueCurrent().getValue();
        inputs.appliedVolts = leftMotor.getDutyCycle().getValue() * rightMotor.getSupplyVoltage().getValue();
        inputs.velocityRadPerS = encoder.getVelocity().getValue() * 2 * Math.PI;

        inputs.desiredAcceleration = (leftMotor.getClosedLoopReferenceSlope().getValue()
                - inputs.desiredVelocity) / 2.0;
        inputs.currentAcceleration = leftMotor.getAcceleration().getValue();

        inputs.desiredVelocity = leftMotor.getClosedLoopReferenceSlope().getValue();
        inputs.currentVelocity = leftMotor.getVelocity().getValue();

        inputs.pivotAngle = leftMotor.getPosition().getValue() - offset;
        inputs.currentDesiredAngle = leftMotor.getClosedLoopReference().getValue() - offset;

        inputs.desiredAngle = goal.position - offset;

        if (leftMotor.getPosition().getValueAsDouble() > 0.9) {
            offset = 1;
        }
    }

    @Override
    public void setAngle(double angle) {
        // setpoint = new
        // TrapezoidProfile.State(leftMotor.getPosition().getValueAsDouble(),
        // leftMotor.getVelocity().getValueAsDouble());
        goal = new TrapezoidProfile.State(Units.radiansToRotations(angle) + offset, 0);
    }

    @Override
    public void disable() {
        leftMotor.disable();
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
