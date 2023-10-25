package com.team2383.robot.subsystems.drivetrain;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2383.lib.math.Conversions;
import com.team2383.lib.swerve.IAbsoluteEncoder;
import com.team2383.robot.subsystems.drivetrain.DriveConstants.ModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleIOFalcon500 implements SwerveModuleIO {
    private final TalonFX m_angleMotor;
    private final TalonFX m_driveMotor;

    private final IAbsoluteEncoder m_angleEncoder;

    private final VelocityVoltage m_velocityOut = new VelocityVoltage(0, true, 0, 0, false);
    private final PositionVoltage m_positionOut = new PositionVoltage(0, true, 0, 0, false);

    private Rotation2d m_lastAngle = new Rotation2d();

    private final Rotation2d m_angleOffset;

    public SwerveModuleIOFalcon500(ModuleConstants constants, IAbsoluteEncoder angleEncoder, String CANbus) {
        this.m_angleMotor = new TalonFX(constants.kAngleMotorID, CANbus);
        this.m_driveMotor = new TalonFX(constants.kDriveMotorID, CANbus);

        this.m_angleEncoder = angleEncoder;

        this.m_angleOffset = constants.kAngleOffset;

        /* Motor Config */
        configAngleMotor(constants);
        configDriveMotor(constants);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.angleRad = Math.toRadians(Conversions.rotationsToDegrees(
                m_angleMotor.getRotorPosition().getValue(),
                DriveConstants.kAngleGearRatio));
        inputs.absoluteAngleRad = getAbsolute().getRadians();
        inputs.velocityMPS = Conversions.RPSToMPS(
                m_driveMotor.getVelocity().getValue(),
                DriveConstants.kDriveWheelCircumferenceMeters,
                DriveConstants.kDriveGearRatio);
        inputs.drivePositionM = Conversions.rotationsToMeters(m_driveMotor.getPosition().getValue(),
                DriveConstants.kDriveWheelCircumferenceMeters,
                DriveConstants.kDriveGearRatio);
        inputs.appliedVoltsDrive = m_driveMotor.getDutyCycle().getValue() * m_driveMotor.getSupplyVoltage().getValue();
        inputs.appliedVoltsAngle = m_angleMotor.getDutyCycle().getValue() * m_angleMotor.getSupplyVoltage().getValue();
        inputs.currentDrive = m_driveMotor.getStatorCurrent().getValue();
        inputs.currentAngle = m_angleMotor.getStatorCurrent().getValue();
    }

    @Override
    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToRotations(
                getAbsolute().getDegrees() - m_angleOffset.getDegrees(),
                DriveConstants.kAngleGearRatio);
        m_angleMotor.setRotorPosition(absolutePosition);
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        setSpeed(desiredState);
        setAngle(desiredState);
    }

    private void configAngleMotor(ModuleConstants constants) {
        m_angleMotor.getConfigurator().apply(constants.kHardwareConfigs.kAngleMotorConfigs);
        m_angleMotor.setInverted(true);
        resetToAbsolute();
    }

    private void configDriveMotor(ModuleConstants constants) {
        m_driveMotor.getConfigurator().apply(constants.kHardwareConfigs.kDriveMotorConfigs);
        m_driveMotor.setInverted(true);
        m_driveMotor.setRotorPosition(0);
    }

    private void setSpeed(SwerveModuleState desiredState) {
        var desiredRPS = Conversions.MPSToFalconRPS(desiredState.speedMetersPerSecond,
                DriveConstants.kDriveWheelCircumferenceMeters, DriveConstants.kDriveGearRatio);
        m_driveMotor.setControl(m_velocityOut.withVelocity(desiredRPS));

    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d desiredAngle = (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kMaxSpeed * 0.01))
                ? m_lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        m_angleMotor.setControl(m_positionOut.withPosition(
                Conversions.degreesToRotations(desiredAngle.getDegrees(), DriveConstants.kAngleGearRatio)));
        m_lastAngle = desiredAngle;
    }

    private Rotation2d getAbsolute() {
        return m_angleEncoder.getAbsoluteAngle();
    }

}
