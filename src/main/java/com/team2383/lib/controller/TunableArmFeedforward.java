package com.team2383.lib.controller;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class TunableArmFeedforward implements Sendable {
    public double ks;
    public double kg;
    public double kv;
    public double ka;

    public TunableArmFeedforward(double kS, double kG, double kV, double kA) {
        this.ks = kS;
        this.kg = kG;
        this.kv = kV;
        this.ka = kA;
    }

    public double calculate(double positionRadians, double velocity, double acceleration) {
        return ks * Math.signum(velocity) + kg * Math.cos(positionRadians) + kv * velocity + ka * acceleration;
    }

    public double calculate(double positionRadians, double velocity) {
        return calculate(positionRadians, velocity, 0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("TunableArmFeedforward");
        builder.addDoubleProperty("kS", () -> ks, (value) -> ks = value);
        builder.addDoubleProperty("kG", () -> kg, (value) -> kg = value);
        builder.addDoubleProperty("kV", () -> kv, (value) -> kv = value);
        builder.addDoubleProperty("kA", () -> ka, (value) -> ka = value);
    }
}
