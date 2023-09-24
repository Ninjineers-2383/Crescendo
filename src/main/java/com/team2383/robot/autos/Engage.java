package com.team2383.robot.autos;

import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Engage extends CommandBase {
    private final DrivetrainSubsystem m_drivetrain;
    private final PIDController m_pid = new PIDController(0.015, 0, 0);

    public Engage(DrivetrainSubsystem drivetrain, boolean invert) {
        this.m_drivetrain = drivetrain;

        this.m_pid.setTolerance(0.1);
        this.m_pid.setSetpoint(0);
        this.m_pid.setP(m_pid.getP() * (invert ? -1 : 1));

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.drive(new Translation2d(-m_pid.calculate(m_drivetrain.getRoll()), 0), new Rotation2d(0), false,
                new Translation2d());
    }

    @Override
    public boolean isFinished() {
        return m_pid.atSetpoint();
    }
}
