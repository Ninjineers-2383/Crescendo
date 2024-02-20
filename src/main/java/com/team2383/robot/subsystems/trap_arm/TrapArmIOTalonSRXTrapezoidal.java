package com.team2383.robot.subsystems.trap_arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team2383.lib.controller.TunableArmFeedforward;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TrapArmIOTalonSRXTrapezoidal implements TrapArmIO {
    private final TalonSRX pivot = new TalonSRX(TrapArmConstants.kPivotID);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            5, 7);

    private TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private PIDController m_feedbackController = TrapArmConstants.kFeedbackController;
    private TunableArmFeedforward m_feedforwardController = TrapArmConstants.kFeedforwardController;

    public TrapArmIOTalonSRXTrapezoidal() {
        setPIDController(TrapArmConstants.kFeedbackController);
        setFeedforward(TrapArmConstants.kFeedforwardController);

        pivot.configFactoryDefault();
        pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        pivot.setSelectedSensorPosition(0);

        goal = new TrapezoidProfile.State(pivot.getSelectedSensorPosition(), 0);
        setpoint = goal;
    }

    public void updateInputs(TrapArmIOInputs inputs) {
        setpoint = profile.calculate(0.02, setpoint, goal);

        pivot.set(ControlMode.PercentOutput,
                m_feedforwardController.calculate(setpoint.position, setpoint.velocity)
                        + m_feedbackController.calculate(getAngle(), setpoint.position));

        inputs.current = pivot.getSupplyCurrent();
        inputs.appliedVolts = pivot.getMotorOutputVoltage();
        inputs.velocityRadPerS = getVelocity();

        inputs.desiredVelocity = goal.velocity;
        inputs.currentVelocity = setpoint.velocity;

        inputs.pivotAngle = getAngle();
        inputs.currentDesiredAngle = setpoint.position;

        inputs.desiredAngle = goal.position;
    }

    @Override
    public void setAngle(double angle) {
        goal = new TrapezoidProfile.State(angle, 0);
    }

    @Override
    public void setPIDController(PIDController controller) {
        m_feedbackController.setP(controller.getP());
        m_feedbackController.setI(controller.getI());
        m_feedbackController.setD(controller.getD());
    }

    @Override
    public void setFeedforward(TunableArmFeedforward feedforward) {
        m_feedforwardController = feedforward;
    }

    private double getAngle() {
        return pivot.getSelectedSensorPosition() / 4096.0 * 2 * Math.PI;
    }

    private double getVelocity() {
        return pivot.getSelectedSensorVelocity() / 4096.0 * 2 * Math.PI * 10;
    }
}
