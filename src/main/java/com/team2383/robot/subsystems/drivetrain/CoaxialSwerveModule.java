package com.team2383.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.team2383.lib.util.OnboardModuleState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CoaxialSwerveModule implements Sendable {
    private final SwerveModuleIO m_io;
    private final SwerveModuleIOInputsAutoLogged m_inputs = new SwerveModuleIOInputsAutoLogged();

    private final String m_logName;

    private SwerveModuleState m_desiredState = new SwerveModuleState();

    public CoaxialSwerveModule(SwerveModuleIO io, String name) {
        this.m_io = io;
        this.m_logName = "Swerve Module/" + name;
    }

    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.getInstance().processInputs(m_logName, m_inputs);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        m_desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        m_io.setDesiredState(m_desiredState);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(
                m_inputs.angleRad);
    }

    public void resetToAbsolute() {
        m_io.resetToAbsolute();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                m_inputs.velocityMPS,
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_inputs.drivePositionM,
                getAngle());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Desired MPS", () -> m_desiredState.speedMetersPerSecond, null);
        builder.addDoubleProperty("Desired Angle", () -> m_desiredState.angle.getDegrees(), null);
        builder.addDoubleProperty("Actual MPS", () -> m_inputs.velocityMPS, null);
        builder.addDoubleProperty("Actual Angle", () -> Math.toDegrees(m_inputs.angleRad), null);
    }
}
