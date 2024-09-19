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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/Gains/kP", PivotConstants.kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/Gains/kI", PivotConstants.kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/Gains/kD", PivotConstants.kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/Gains/kS", PivotConstants.kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/Gains/kV", PivotConstants.kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/Gains/kA", PivotConstants.kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/Gains/kG", PivotConstants.kGains.kG());
    private static final LoggedTunableNumber kSpring = new LoggedTunableNumber("Pivot/Gains/kSpring",
            PivotConstants.kGains.kSpring());
    private static final LoggedTunableNumber kBacklash = new LoggedTunableNumber("Pivot/Gains/kBacklash",
            PivotConstants.kGains.kBacklash());

    private final LoggedTunableNumber kMaxAngleDegrees = new LoggedTunableNumber("Pivot/Bounds/MaxAngle",
            PivotConstants.kMaxAngleDegrees);

    private final LoggedTunableNumber kMinAngleDegrees = new LoggedTunableNumber("Pivot/Bounds/MinAngle",
            PivotConstants.kMinAngleDegrees);

    private final LoggedTunableNumber kMaxVelocity = new LoggedTunableNumber("Pivot/TrapezoidalConstraints/MaxVelocity",
            PivotConstants.kMaxVelo);
    private final LoggedTunableNumber kMaxAccel = new LoggedTunableNumber(
            "Pivot/TrapezoidalConstraints/MaxAcceleration",
            PivotConstants.kMaxAccel);

    private final LoggedTunableNumber kPivotTolerance = new LoggedTunableNumber("Pivot/ToleranceDegrees", 1.0);

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
    private TrapezoidProfile.State forwardGoal = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private LashState lashState = LashState.Reverse;

    public enum LashState {
        Forward, Reverse
    }

    public PivotSubsystem(PivotIO io) {
        this.io = io;

        goal = new TrapezoidProfile.State(getAngle().getRotations(), 0);
        forwardGoal = goal;
        setpoint = goal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        if (Math.abs(inputs.absoluteEncoderPositionRot - setpoint.position) > (10 / 360.0)
                && lashState == LashState.Reverse && DriverStation.isEnabled()) {
            lashState = LashState.Forward;
            setpoint = new TrapezoidProfile.State(inputs.absoluteEncoderPositionRot, 0.0);
        }

        if (lashState == LashState.Forward) {
            setpoint = profile.calculate(0.02, setpoint, forwardGoal);
            if (setpoint.equals(forwardGoal)) {
                lashState = LashState.Reverse;
                setpoint = new TrapezoidProfile.State(inputs.absoluteEncoderPositionRot, 0);
            }
        }
        if (lashState == LashState.Reverse) {
            setpoint = profile.calculate(0.02, setpoint,
                    new TrapezoidProfile.State(
                            MathUtil.clamp(
                                    goal.position,
                                    Units.degreesToRotations(kMinAngleDegrees.get()),
                                    Units.degreesToRotations(kMaxAngleDegrees.get())),
                            0.0));
        }

        io.setAngleRot(setpoint.position, setpoint.velocity, lashState);

        Logger.recordOutput("Pivot/Goal/Position", goal.position);
        Logger.recordOutput("Pivot/Goal/Velocity", goal.velocity);
        Logger.recordOutput("Pivot/Setpoint/Position", setpoint.position);
        Logger.recordOutput("Pivot/Setpoint/Velocity", setpoint.velocity);
        Logger.recordOutput("Pivot/State", lashState.toString());

        leftMotorDisconnected.set(!inputs.leftMotorConnected);
        rightMotorDisconnected.set(!inputs.rightMotorConnected);
        encoderDisconnected.set(!inputs.encoderConnected);

        LoggedTunableNumber.ifChanged(hashCode(), (pid) -> io.setPIDController(pid[0], pid[1], pid[2]), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), (ff) -> io.setFeedforward(ff[0], ff[1], ff[2], ff[3], ff[4]), kS,
                kV, kA, kG, kSpring);

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
        lashState = LashState.Forward;
        forwardGoal = new TrapezoidProfile.State(
                MathUtil.clamp(
                        goal.position > inputs.absoluteEncoderPositionRot
                                ? goal.position + (10.0 / 360.0)
                                : goal.position,
                        Units.degreesToRotations(kMinAngleDegrees.get()),
                        Units.degreesToRotations(kMaxAngleDegrees.get())),
                0.0);
    }

    public void setPosition(double angleRads) {
        if (Units.radiansToRotations(angleRads) == goal.position)
            return;
        goal = new TrapezoidProfile.State(Units.radiansToRotations(angleRads), 0);
        if (Math.abs(angleRads - Units.rotationsToRadians(inputs.absoluteEncoderPositionRot)) > 4)
            lashState = LashState.Forward;
        forwardGoal = new TrapezoidProfile.State(
                MathUtil.clamp(
                        goal.position > inputs.absoluteEncoderPositionRot
                                ? goal.position + (kBacklash.get() / 360.0)
                                : goal.position,
                        Units.degreesToRotations(kMinAngleDegrees.get()),
                        Units.degreesToRotations(kMaxAngleDegrees.get())),
                0.0);

    }

    public boolean isFinished(double toleranceDegrees) {
        return Math.abs(inputs.absoluteEncoderPositionRot - goal.position) < ((toleranceDegrees) / 360.0);
    }

    public boolean isFinished() {
        return isFinished(kPivotTolerance.get());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(inputs.absoluteEncoderPositionRot);
    }

    public Rotation2d getDesiredAngle() {
        return Rotation2d.fromRotations(inputs.desiredPositionRot);
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
