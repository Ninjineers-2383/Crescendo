package com.team2383.robot.subsystems.drivetrain.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Supplier;

public class GyroIOSim implements GyroIO {
    private final Supplier<ChassisSpeeds> robotSpeeds;

    private Rotation2d heading = new Rotation2d();

    public GyroIOSim(Supplier<ChassisSpeeds> robotSpeeds) {
        this.robotSpeeds = robotSpeeds;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        heading = heading.plus(Rotation2d.fromRadians(robotSpeeds.get().omegaRadiansPerSecond * 0.02));

        inputs.connected = true; // TODO: Figure out how to get connected status
        inputs.headingDeg = heading.getDegrees();
    }

    @Override
    public void resetHeading() {
        // m_gyro.setAngleAdjustment(-m_gyro.getAngle());
    }

    @Override
    public void setHeading(Rotation2d heading) {
        this.heading = heading;
    }
}