package com.team2383.lib.swerve;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class AbsoluteCancoder implements IAbsoluteEncoder {
    private final CANcoder encoder;

    public AbsoluteCancoder(int id, String canbus, CANcoderConfiguration config) {
        this.encoder = new CANcoder(id, canbus);

        this.encoder.getConfigurator().apply(config);
    }

    @Override
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(this.encoder.getAbsolutePosition().getValue());
    }
}
