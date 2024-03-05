package com.team2383.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.team2383.lib.util.mechanical_advantage.Alert;
import com.team2383.lib.util.mechanical_advantage.LoggedTunableNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/Gains/kP", PivotConstants.kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/Gains/kI", PivotConstants.kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/Gains/kD", PivotConstants.kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/Gains/kS", PivotConstants.kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/Gains/kV", PivotConstants.kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/Gains/kA", PivotConstants.kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/Gains/kG", PivotConstants.kGains.kG());

    private final LoggedTunableNumber kMaxAngleDegrees = new LoggedTunableNumber("Pivot/Bounds/MaxAngle",
            PivotConstants.kMaxAngleDegrees);

    private final LoggedTunableNumber kMinAngleDegrees = new LoggedTunableNumber("Pivot/Bounds/MinAngle",
            PivotConstants.kMinAngleDegrees);

    private final LoggedTunableNumber kMaxVelocity = new LoggedTunableNumber("Pivot/TrapezoidalConstraints/MaxVelocity",
            PivotConstants.kMaxVelo);
    private final LoggedTunableNumber kMaxAccel = new LoggedTunableNumber(
            "Pivot/TrapezoidalConstraints/MaxAcceleration",
            PivotConstants.kMaxAccel);

    private final Alert leftMotorDisconnected = new Alert(
            "Pivot left motor disconnected! CAN ID: " + PivotConstants.kLeftMotorID, Alert.AlertType.WARNING);
    private final Alert rightMotorDisconnected = new Alert(
            "Pivot right motor disconnected! CAN ID: " + PivotConstants.kRightMotorID, Alert.AlertType.WARNING);
    private final Alert encoderDisconnected = new Alert(
            "Pivot encoder disconnected! CAN ID: " + PivotConstants.kEncoderID, Alert.AlertType.WARNING);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            PivotConstants.kMaxVelo, PivotConstants.kMaxAccel);

    private TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public PivotSubsystem(PivotIO io) {
        this.io = io;

        goal = new TrapezoidProfile.State(getAngle().getRotations(), 0);
        setpoint = goal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        inputs.desiredPositionRot = goal.position;
        Logger.processInputs("Pivot", inputs);

        setpoint = profile.calculate(0.02, setpoint,
                new TrapezoidProfile.State(
                        MathUtil.clamp(
                                goal.position,
                                Units.degreesToRotations(kMinAngleDegrees.get()),
                                Units.degreesToRotations(kMaxAngleDegrees.get())),
                        0.0));

        io.setAngleRot(setpoint.position, setpoint.velocity);

        leftMotorDisconnected.set(!inputs.leftMotorConnected);
        rightMotorDisconnected.set(!inputs.rightMotorConnected);
        encoderDisconnected.set(!inputs.encoderConnected);

        LoggedTunableNumber.ifChanged(hashCode(), (pid) -> io.setPIDController(pid[0], pid[1], pid[2]), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), (ff) -> io.setFeedforward(ff[0], ff[1], ff[2], ff[3]), kS,
                kV, kA, kG);

        LoggedTunableNumber.ifChanged(hashCode(),
                (constraints) -> profile = new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(constraints[0], constraints[1])),
                kMaxVelocity, kMaxAccel);
    }

    public void setVoltage(Measure<Voltage> voltage) {
        io.setVoltage(voltage.in(Volts));
    }

    public void incrementPosition(double deltaAngle) {
        goal.position += deltaAngle;
    }

    public void setPosition(double angleRads) {
        goal = new TrapezoidProfile.State(Units.radiansToRotations(angleRads), 0);
    }

    public boolean isFinished() {
        return Math.abs(inputs.absoluteEncoderPositionRot - inputs.desiredPositionRot) < (1.2) / 360;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(inputs.absoluteEncoderPositionRot);
    }

    public double getVelocityRadPerSec() {
        return Units.rotationsToRadians(inputs.desiredVelocityRotPerSec);
    }

    public void disable() {
        io.disable();
    }

    public double getVoltage() {
        return inputs.appliedVolts[0];
    }
}
