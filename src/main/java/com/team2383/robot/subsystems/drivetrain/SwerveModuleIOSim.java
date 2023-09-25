package com.team2383.robot.subsystems.drivetrain;

import com.team2383.lib.math.Conversions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final FlywheelSim driveSim;
    private final FlywheelSim angleSim;

    private final PIDController driveController = new PIDController(0.27135, 0, 0);
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0.55, 0.015968);
    private final PIDController angleController = new PIDController(4, 0, 0);

    private double absolutePosition = Math.random();

    private double driveRotations = 0.0;

    private double desiredRps = 0.0;
    private double desiredRotations = 0.0;

    public SwerveModuleIOSim() {
        driveSim = new FlywheelSim(DCMotor.getFalcon500(1), DriveConstants.kDriveGearRatio, 1.5E-5);
        angleSim = new FlywheelSim(DCMotor.getFalcon500(1), DriveConstants.kAngleGearRatio, 1E-7);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        double driveVoltage = driveFeedforward.calculate(desiredRps)
                + driveController.calculate(driveSim.getAngularVelocityRPM() / 60.0, desiredRps);
        double angleVoltage = angleController.calculate(absolutePosition, desiredRotations);

        driveSim.setInputVoltage(driveVoltage);
        angleSim.setInputVoltage(angleVoltage);

        driveSim.update(0.02);
        angleSim.update(0.02);

        absolutePosition += angleSim.getAngularVelocityRPM() / 60.0 * 0.02;
        driveRotations += driveSim.getAngularVelocityRPM() / 60.0 * 0.02;

        inputs.velocityMPS = Conversions.RPSToMPS(driveSim.getAngularVelocityRPM() / 60.0,
                DriveConstants.kDriveWheelCircumferenceMeters, DriveConstants.kDriveGearRatio);
        inputs.angleRad = Math
                .toRadians(Conversions.rotationsToDegrees(absolutePosition, DriveConstants.kAngleGearRatio));
        inputs.drivePositionM = Conversions.rotationsToMeters(driveRotations,
                DriveConstants.kDriveWheelCircumferenceMeters, DriveConstants.kDriveGearRatio);
    }

    @Override
    public void resetToAbsolute() {
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        desiredRps = Conversions.MPSToFalconRPS(desiredState.speedMetersPerSecond,
                DriveConstants.kDriveWheelCircumferenceMeters, DriveConstants.kDriveGearRatio);

        desiredRotations = Conversions.degreesToRotations(desiredState.angle.getDegrees(),
                DriveConstants.kAngleGearRatio);
    }

}