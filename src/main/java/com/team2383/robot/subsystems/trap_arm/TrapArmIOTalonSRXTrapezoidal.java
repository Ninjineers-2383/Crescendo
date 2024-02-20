package com.team2383.robot.subsystems.trapArm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class TrapArmIOTalonSRXTrapezoidal implements TrapArmIO {
    private final TalonSRX pivot = new TalonSRX(TrapArmConstants.kPivotID);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            5, 7);

    private TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private PIDController m_feedbackController = TrapArmConstants.kFeedbackController;
    private ArmFeedforward m_feedforwardController = TrapArmConstants.kFeedforwardController;

    private double offset = 0;

    public TrapArmIOTalonSRXTrapezoidal() {
        setPIDController(TrapArmConstants.kFeedbackController);
        setFeedforward(TrapArmConstants.kFeedforwardController);

        goal = new TrapezoidProfile.State(pivot.getSelectedSensorPosition(), 0);
        setpoint = goal;
    }

    public void updateInputs(TrapArmIOInputs inputs) {
        setpoint = profile.calculate(0.02, setpoint, goal);

        pivot.set(ControlMode.PercentOutput, (m_feedforwardController.calculate(setpoint.position, setpoint.velocity)
                + m_feedbackController.calculate(pivot.getSelectedSensorPosition(), setpoint.position) / 12));

        inputs.current = pivot.getStatorCurrent();
        inputs.appliedVolts = pivot.getMotorOutputVoltage();
        inputs.velocityRadPerS = pivot.getSelectedSensorVelocity();

        inputs.desiredVelocity = setpoint.velocity;
        inputs.currentVelocity = pivot.getSelectedSensorVelocity();

        inputs.pivotAngle = pivot.getSelectedSensorPosition();
        inputs.currentDesiredAngle = setpoint.position;

        inputs.desiredAngle = goal.position - offset;
    }

    @Override
    public void setAngle(double angle) {
        goal = new TrapezoidProfile.State(Units.radiansToRotations(angle) + offset, 0);
    }

    @Override
    public void setPIDController(PIDController controller) {
        m_feedbackController = controller;
    }

    @Override
    public void setFeedforward(ArmFeedforward feedforward) {
        m_feedforwardController = feedforward;
    }

}
