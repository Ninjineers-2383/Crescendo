package com.team2383.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim implements PivotIO {
    private DCMotorSim motor = new DCMotorSim(DCMotor.getFalcon500(2), PivotConstants.kPivotMotorGearRatio, 0.01);

    private double desiredAngle = 0;
    private PIDController controller = new PIDController(PivotConstants.kGains.kP(), PivotConstants.kGains.kI(),
            PivotConstants.kGains.kD());

    private ArmFeedforward feedforward = new ArmFeedforward(PivotConstants.kGains.kS(), PivotConstants.kGains.kV(),
            PivotConstants.kGains.kA());

    public PivotIOSim() {

    }

    public void updateInputs(PivotIOInputs inputs) {
        motor.setInputVoltage(controller.calculate(motor.getAngularPositionRotations(), desiredAngle)
                + feedforward.calculate(desiredAngle, 3));

    }

    @Override
    public void setAngleRot(double angle, double velocity) {
        desiredAngle = angle;
    }
}
