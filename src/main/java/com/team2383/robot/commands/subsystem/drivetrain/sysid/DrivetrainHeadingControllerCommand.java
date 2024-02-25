package com.team2383.robot.commands.subsystem.drivetrain.sysid;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainHeadingControllerCommand extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier speed;
    private final BooleanSupplier resetSetpoint;

    private final ProfiledPIDController pidController = new ProfiledPIDController(5.0, 0, 0,
            new Constraints(4 * Math.PI, 4 * Math.PI));

    public DrivetrainHeadingControllerCommand(DrivetrainSubsystem drivetrain, DoubleSupplier speed,
            BooleanSupplier resetSetpoint) {
        this.drivetrain = drivetrain;
        this.speed = speed;
        this.resetSetpoint = resetSetpoint;

        pidController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        SmartDashboard.putData("Drivetrain Heading PID", pidController);
    }

    @Override
    public void execute() {
        drivetrain.setHeadingPID(pidController);

        drivetrain.drive(new ChassisSpeeds(0, 0, -speed.getAsDouble()), true, true);

        if (resetSetpoint.getAsBoolean()) {
            drivetrain.setHeading(drivetrain.getHeading());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.endManualHeadingControl();
    }

}
