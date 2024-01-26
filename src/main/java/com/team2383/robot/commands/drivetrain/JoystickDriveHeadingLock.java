package com.team2383.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.team2383.lib.math.ThrottleSoftener;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class JoystickDriveHeadingLock extends Command {
    private final DrivetrainSubsystem m_drivetrain;

    private final Supplier<Rotation2d> m_rotSupply;
    private final Supplier<Translation2d> m_moveSupply;
    private final BooleanSupplier m_fieldRelative;
    private final IntSupplier m_hatSupplier;
    private Rotation2d m_storedRotation = new Rotation2d();

    private final PIDController m_controller = new PIDController(4, 0, 0);
    private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(8);
    private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(8);
    private final SlewRateLimiter m_oRateLimiter = new SlewRateLimiter(15);

    private boolean prev_zero = false;

    public JoystickDriveHeadingLock(DrivetrainSubsystem drivetrain, Supplier<Translation2d> moveSupplier,
            Supplier<Rotation2d> rotation, BooleanSupplier fieldRelative, IntSupplier hatSupplier) {
        m_drivetrain = drivetrain;

        m_controller.enableContinuousInput(0, 2 * Math.PI);
        m_moveSupply = moveSupplier;
        m_rotSupply = rotation;
        m_fieldRelative = fieldRelative;
        m_hatSupplier = hatSupplier;

        addRequirements(m_drivetrain);

    }

    @Override
    public void initialize() {
        m_storedRotation = m_drivetrain.getHeading();
    }

    @Override
    public void execute() {
        Translation2d move = m_moveSupply.get();
        double x = -ThrottleSoftener.soften(move.getX())
                * DriveConstants.kMaxSpeed;
        double y = -ThrottleSoftener.soften(move.getY())
                * DriveConstants.kMaxSpeed;
        Rotation2d omega = Rotation2d
                .fromRadians(m_oRateLimiter.calculate(-ThrottleSoftener.soften(m_rotSupply.get().getRadians())));

        Rotation2d rotVelocity;

        if (omega.getRadians() == 0 && prev_zero == false) {
            m_storedRotation = m_drivetrain.getHeading();
        }

        if (omega.getRadians() == 0 && move.getX() + move.getY() != 0) {
            rotVelocity = new Rotation2d(
                    m_controller.calculate(m_drivetrain.getHeading().getRadians(), m_storedRotation.getRadians()));
            prev_zero = true;
        } else {
            rotVelocity = omega;
            prev_zero = false;
        }

        int hatPosition = m_hatSupplier.getAsInt();

        SmartDashboard.putNumber("Omega", omega.getDegrees());
        SmartDashboard.putNumber("Stored Rot", m_storedRotation.getDegrees());
        SmartDashboard.putNumber("Rot Velocity", rotVelocity.getDegrees());

        m_drivetrain.drive(
                new Translation2d(
                        m_xRateLimiter.calculate(x),
                        m_yRateLimiter.calculate(y)),
                rotVelocity,
                m_fieldRelative.getAsBoolean(),
                getCenterOfRotation(hatPosition));
    }

    private Translation2d getCenterOfRotation(int hatPosition) {
        if (!(hatPosition >= 0 && hatPosition <= 360)) {
            return new Translation2d(0, 0);
        }
        // Allows rotating around the swerve modules
        Translation2d pos = new Translation2d((Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 3, 0);
        pos = pos.rotateBy(Rotation2d.fromDegrees(-hatPosition));
        return new Translation2d((Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 12, 0).plus(pos);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
