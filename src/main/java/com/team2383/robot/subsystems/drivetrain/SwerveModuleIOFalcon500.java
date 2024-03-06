package com.team2383.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2383.lib.math.Conversions;
import com.team2383.lib.swerve.IAbsoluteEncoder;
import com.team2383.robot.subsystems.drivetrain.DriveConstants.ModuleConstants;
import com.team2383.robot.subsystems.orchestra.OrchestraContainer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleIOFalcon500 implements SwerveModuleIO {
    private final TalonFX m_angleMotor;
    private final TalonFX m_driveMotor;

    private final IAbsoluteEncoder m_angleEncoder;

    private final VelocityVoltage m_velocityOut = new VelocityVoltage(0);
    private final PositionVoltage m_positionOut = new PositionVoltage(0);

    private Rotation2d m_lastAngle = new Rotation2d();

    private final Rotation2d m_angleOffset;

    private boolean voltageControl = false;

    private final StatusSignal<Double> m_angleRotorPosition;
    private final StatusSignal<Double> m_driveVelocity;
    private final StatusSignal<Double> m_angleVelocity;
    private final StatusSignal<Double> m_drivePosition;
    private final StatusSignal<Double> m_driveDutyCycle;
    private final StatusSignal<Double> m_angleDutyCycle;
    private final StatusSignal<Double> m_driveVoltage;
    private final StatusSignal<Double> m_angleVoltage;
    private final StatusSignal<Double> m_driveCurrent;
    private final StatusSignal<Double> m_angleCurrent;

    public SwerveModuleIOFalcon500(ModuleConstants constants, String CANbus) {
        this.m_angleMotor = new TalonFX(constants.kAngleMotorID, CANbus);
        this.m_driveMotor = new TalonFX(constants.kDriveMotorID, CANbus);

        OrchestraContainer.getInstance().addMotor(m_angleMotor);
        OrchestraContainer.getInstance().addMotor(m_driveMotor);

        this.m_angleEncoder = constants.kEncoder;

        this.m_angleOffset = constants.kAngleOffset;

        /* Motor Config */
        configAngleMotor(constants);
        configDriveMotor(constants);

        m_angleRotorPosition = m_angleMotor.getRotorPosition();
        m_driveVelocity = m_driveMotor.getVelocity();
        m_angleVelocity = m_angleMotor.getVelocity();
        m_drivePosition = m_driveMotor.getPosition();
        m_driveDutyCycle = m_driveMotor.getDutyCycle();
        m_angleDutyCycle = m_angleMotor.getDutyCycle();
        m_driveVoltage = m_driveMotor.getSupplyVoltage();
        m_angleVoltage = m_angleMotor.getSupplyVoltage();
        m_driveCurrent = m_driveMotor.getStatorCurrent();
        m_angleCurrent = m_angleMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                m_angleRotorPosition,
                m_driveVelocity,
                m_angleVelocity,
                m_drivePosition,
                m_driveDutyCycle,
                m_angleDutyCycle,
                m_driveVoltage,
                m_angleVoltage,
                m_driveCurrent,
                m_angleCurrent);

        m_driveMotor.optimizeBusUtilization(1.0);
        m_angleMotor.optimizeBusUtilization(1.0);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.driveMotorConnected = BaseStatusSignal.refreshAll(
                m_driveVelocity,
                m_drivePosition,
                m_driveDutyCycle,
                m_driveVoltage,
                m_driveCurrent)
                .isOK();

        inputs.angleEncoderConnected = m_angleEncoder.getConnected();

        inputs.angleMotorConnected = BaseStatusSignal.refreshAll(
                m_angleRotorPosition,
                m_angleVelocity,
                m_angleDutyCycle,
                m_angleVoltage,
                m_angleCurrent)
                .isOK();

        inputs.angleRad = Math.toRadians(Conversions.rotationsToDegrees(
                m_angleRotorPosition.getValue(),
                DriveConstants.kAngleGearRatio));
        inputs.absoluteAngleRad = getAbsolute().getRadians();
        inputs.driveVelocityMPS = Conversions.RPSToMPS(
                m_driveVelocity.getValue(),
                DriveConstants.kDriveWheelCircumferenceMeters,
                DriveConstants.kDriveGearRatio);
        inputs.azimuthVelocityRPM = Conversions.falconToRPM(
                m_angleVelocity.getValue(),
                DriveConstants.kAngleGearRatio);
        inputs.drivePositionM = Conversions.rotationsToMeters(m_drivePosition.getValue(),
                DriveConstants.kDriveWheelCircumferenceMeters,
                DriveConstants.kDriveGearRatio);
        inputs.appliedVoltsDrive = m_driveDutyCycle.getValue() * m_driveVoltage.getValue();
        inputs.appliedVoltsAngle = m_angleDutyCycle.getValue() * m_angleVoltage.getValue();
        inputs.currentDrive = m_driveCurrent.getValue();
        inputs.currentAngle = m_angleCurrent.getValue();
    }

    @Override
    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToRotations(
                getAbsolute().getDegrees() - m_angleOffset.getDegrees(),
                DriveConstants.kAngleGearRatio);
        m_angleMotor.setPosition(absolutePosition);
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        setSpeed(desiredState);
        setAngle(desiredState);
    }

    @Override
    public void setVoltage(double voltage) {
        voltageControl = true;
        // m_angleMotor.setVoltage(voltage);
        m_driveMotor.setVoltage(voltage);
        setAngle(new SwerveModuleState(10, new Rotation2d()));
    }

    private void configAngleMotor(ModuleConstants constants) {
        m_angleMotor.getConfigurator().apply(constants.kHardwareConfigs.kAngleMotorConfigs);
        m_angleMotor.setInverted(!constants.invertAzimuth);
        resetToAbsolute();
    }

    private void configDriveMotor(ModuleConstants constants) {
        m_driveMotor.getConfigurator().apply(constants.kHardwareConfigs.kDriveMotorConfigs);
        m_driveMotor.setInverted(constants.invertDrive);
        m_driveMotor.setPosition(0);
    }

    private void setSpeed(SwerveModuleState desiredState) {
        if (!voltageControl) {
            var desiredRPS = Conversions.MPSToFalconRPS(desiredState.speedMetersPerSecond,
                    DriveConstants.kDriveWheelCircumferenceMeters, DriveConstants.kDriveGearRatio);
            m_driveMotor.setControl(m_velocityOut.withVelocity(desiredRPS));
        }
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
