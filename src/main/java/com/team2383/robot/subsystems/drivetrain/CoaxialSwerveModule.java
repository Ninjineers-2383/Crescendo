package com.team2383.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.team2383.lib.util.OnboardModuleState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CoaxialSwerveModule {
    public final SwerveModuleIO m_io;
    public final SwerveModuleIOInputsAutoLogged m_inputs = new SwerveModuleIOInputsAutoLogged();

    private final String m_logName;

    private SwerveModuleState m_desiredState = new SwerveModuleState();

    // Loop cycle counter for absolute encoder initialization
    private int loop_cycle = 0;

    public CoaxialSwerveModule(SwerveModuleIO io, String name) {
        this.m_io = io;
        this.m_logName = "Swerve Module/" + name;
    }

    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs(m_logName, m_inputs);

        if (loop_cycle == 200) {
            resetToAbsolute();
        }

        loop_cycle++;
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
                m_inputs.driveVelocityMPS,
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_inputs.drivePositionM,
                getAngle());
    }

    public void stop() {
        m_io.stop();
    }

    public void setVoltage(double voltage) {
        m_io.setVoltage(voltage);
    }

}
