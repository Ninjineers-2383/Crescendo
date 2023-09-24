package com.team2383.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteMagEncoder implements IAbsoluteEncoder {
    private final DutyCycleEncoder encoder;

    public AbsoluteMagEncoder(int port) {
        this.encoder = new DutyCycleEncoder(port);
    }

    @Override
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(this.encoder.getAbsolutePosition() - 0.25);
    }
}
