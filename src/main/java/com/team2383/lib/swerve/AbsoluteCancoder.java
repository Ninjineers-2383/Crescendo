package com.team2383.lib.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class AbsoluteCancoder implements IAbsoluteEncoder {
    private final CANcoder encoder;

    public AbsoluteCancoder(int id, String canbus, CANcoderConfiguration config) {
        this.encoder = new CANcoder(id, canbus);

        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        this.encoder.getConfigurator().apply(config);
    }

    @Override
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(this.encoder.getAbsolutePosition().getValue());
    }
}
