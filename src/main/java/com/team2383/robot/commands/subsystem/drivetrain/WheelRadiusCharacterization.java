// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team2383.robot.commands.subsystem.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import com.team2383.lib.util.mechanical_advantage.LoggedTunableNumber;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.function.Supplier;

public class WheelRadiusCharacterization extends Command {
    private final LoggedTunableNumber characterizationSpeed = new LoggedTunableNumber(
            "WheelRadiusCharacterization/SpeedRadsPerSec", 0.1);
    private final double driveRadius = DriveConstants.kDriveRadius;

    private final Supplier<Rotation2d> gyroYawRadsSupplier;

    private final DrivetrainSubsystem drivetrain;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);
    private final Direction omegaDirection;

    private Rotation2d lastGyroYawRads = new Rotation2d();
    private Rotation2d accumGyroYawRads = new Rotation2d();

    private SwerveModulePosition[] startWheelPositions;

    private double currentEffectiveWheelRadius = 0.0;

    public enum Direction {
        CLOCKWISE, COUNTER_CLOCKWISE;
    }

    public WheelRadiusCharacterization(DrivetrainSubsystem drivetrain, Supplier<Rotation2d> gyroYawRadsSupplier,
            Direction direction) {
        this.drivetrain = drivetrain;
        this.gyroYawRadsSupplier = gyroYawRadsSupplier;
        this.omegaDirection = direction;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = gyroYawRadsSupplier.get();
        accumGyroYawRads = new Rotation2d();

        startWheelPositions = drivetrain.getModulePositions();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        // Run drive at velocity
        drivetrain.drive(
                new ChassisSpeeds(0, 0,
                        omegaLimiter.calculate(
                                (omegaDirection == Direction.CLOCKWISE ? 1 : -1) * characterizationSpeed.get())),
                false, false);

        // Get yaw and wheel positions
        accumGyroYawRads = accumGyroYawRads.plus(gyroYawRadsSupplier.get().minus(lastGyroYawRads));
        lastGyroYawRads = gyroYawRadsSupplier.get();
        double averageWheelPosition = 0.0;
        SwerveModulePosition[] wheelPositions = drivetrain.getModulePositions();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositions[i].distanceMeters - startWheelPositions[i].distanceMeters);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads.getRadians() * driveRadius) / averageWheelPosition;
        Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
        Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
        Logger.recordOutput(
                "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
                Units.metersToInches(currentEffectiveWheelRadius));
    }

    @Override
    public void end(boolean interrupted) {
        if (accumGyroYawRads.getRadians() <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        } else {
            System.out.println(
                    "Effective Wheel Radius: "
                            + Units.metersToInches(currentEffectiveWheelRadius)
                            + " inches");
        }
    }
}
