// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.robot.helpers;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that generates and runs trapezoidal motion profiles
 * automatically. The user specifies
 * how to use the current state of the motion profile by overriding the
 * `useState` method.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public abstract class TrapezoidalSubsystemBase extends SubsystemBase {
    private final TrapezoidProfile.Constraints m_constraints;

    private TrapezoidProfile.State m_state;
    private TrapezoidProfile.State m_goal;

    private boolean m_isFinished = true;
    private double m_voltage;
    private double m_desiredVelocity = 0;
    private double m_desiredPosition = 0;
    // private String m_name;

    private boolean m_enabled = true;

    private boolean m_initialized = true;

    private final double m_positionThreshold;

    /**
     * Creates a new TrapezoidProfileSubsystem.
     *
     * @param constraints
     *            The constraints (maximum velocity and acceleration)
     *            for the profiles.
     * @param initialPosition
     *            The initial position of the controlled mechanism when
     *            the subsystem is
     *            constructed.
     * @param period
     *            The period of the main robot loop, in seconds.
     */
    protected TrapezoidalSubsystemBase(
            String name, TrapezoidProfile.Constraints constraints,
            double positionaThreshold, double initialPosition) {

        m_constraints = requireNonNullParam(constraints, "constraints", "TrapezoidProfileSubsystemBase");
        m_state = new TrapezoidProfile.State(initialPosition, 0);
        setGoal(new TrapezoidProfile.State(initialPosition, 0));

        // m_name = name;
        m_positionThreshold = positionaThreshold;
    }

    /**
     * Creates a new TrapezoidProfileSubsystem.
     *
     * @param constraints
     *            The constraints (maximum velocity and acceleration) for
     *            the profiles.
     */
    protected TrapezoidalSubsystemBase(String name, TrapezoidProfile.Constraints constraints,
            double positionThreshold) {
        this(name, constraints, positionThreshold, 0);
    }

    @Override
    public void periodic() {
        // Generate new profile based on current state and goal state
        var profile = new TrapezoidProfile(m_constraints, m_goal, !m_initialized ? getState() : m_state);
        // Gather current command
        m_state = profile.calculate(0.02);

        m_isFinished = profile.isFinished(0.02);

        // Override desired velocity if enabled
        if (m_enabled) {
            m_desiredVelocity = m_isFinished ? 0 : m_state.velocity;
            m_desiredPosition = m_isFinished ? m_goal.position : m_state.position;
        } else {
            m_desiredPosition += m_desiredVelocity * 0.02;
        }

        // Calculate voltage to send to motors
        m_voltage = calculateVoltage(m_desiredVelocity, m_desiredPosition);
        m_voltage = MathUtil.clamp(m_voltage, -12, 12);
        setVoltage(m_voltage);
    }

    /**
     * Sets the goal state for the subsystem.
     *
     * @param goal
     *            The goal state for the subsystem's motion profile.
     */
    protected void setGoal(TrapezoidProfile.State goal) {
        m_goal = goal;
        m_initialized = true;
        enable();
    }

    /** Enable the TrapezoidProfileSubsystem's output. */
    public void enable() {
        m_enabled = true;
    }

    /** Disable the TrapezoidProfileSubsystem's output. */
    public void disable() {
        m_enabled = false;
    }

    /**
     * Return state of subsystem for trapezoidal motion
     * 
     * @return state of subsystem
     */
    protected abstract TrapezoidProfile.State getState();

    /**
     * Set velocity of subsystem
     * 
     * @param velocity
     */
    protected void setVelocity(double velocity) {
        disable();
        m_desiredVelocity = velocity;
    }

    protected abstract double calculateVoltage(double velocity, double position);

    /**
     * Set voltage of motors
     * 
     * @param voltage
     *            to be passed to motors
     */
    protected abstract void setVoltage(double voltage);

    /**
     * Returns if the subsystem is 0.02 seconds away from target
     * 
     * @return boolean isFinished
     */
    public boolean isAtPosition() {
        return m_isFinished && Math.abs(m_goal.position - getState().position) < m_positionThreshold;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.setSmartDashboardType("TrapezoidProfileSubsystem");

        builder.addDoubleProperty("goal", () -> m_goal.position, null);

        builder.addDoubleProperty("position", () -> getState().position, null);

        builder.addDoubleProperty("velocity", () -> getState().velocity, null);

        builder.addDoubleProperty("error", () -> m_goal.position - getState().position, null);
    }
}
