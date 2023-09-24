package com.team2383.lib.controller;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class TunableFeedforward implements Sendable {
    public double ks;
    public double kv;
    public double ka;

    public TunableFeedforward(double kS, double kV, double kA) {
        this.ks = kS;
        this.kv = kV;
        this.ka = kA;
    }

    public double calculate(double velocity, double acceleration) {
        return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
    }

    public double calculate(double velocity) {
        return calculate(velocity, 0);
    }

    public double getKs() {
        return ks;
    }

    public void setKs(double value) {
        ks = value;
    }

    public double getKv() {
        return kv;
    }

    public void setKv(double value) {
        kv = value;
    }

    public double getKa() {
        return ka;
    }

    public void setKa(double value) {
        ka = value;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("TunableFeedforward");
        builder.addDoubleProperty("kS", this::getKs, this::setKs);
        builder.addDoubleProperty("kV", this::getKv, this::setKv);
        builder.addDoubleProperty("kA", this::getKa, this::setKa);
    }
}
