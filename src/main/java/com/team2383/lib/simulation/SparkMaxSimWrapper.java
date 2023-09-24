package com.team2383.lib.simulation;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;

public class SparkMaxSimWrapper extends CANSparkMax {
    private SimDevice m_device;
    private SimDouble m_velocity;
    private SimDouble m_position;
    private SimDouble m_voltage;

    /**
     * Takes in a device ID and a motor type and creates a CANSparkMax wrapper
     * class.
     * All the methods in this file are overriden for our convenience BRIAN.
     * 
     * @param deviceID
     * @param type
     */
    public SparkMaxSimWrapper(int deviceID, MotorType motorType) {
        super(deviceID, motorType);

        m_device = SimDevice.create("Spark Max", this.getDeviceId());

        if (m_device != null) {
            m_position = m_device.createDouble("Position", Direction.kOutput, 0.0);

            m_velocity = m_device.createDouble("Velocity", Direction.kOutput, 0.0);

            m_voltage = m_device.createDouble("Voltage", Direction.kInput, 0.0);

        }
    }

    public void setSimVelocity(double speed) {
        if (m_device != null) {
            m_velocity.set(speed);
        }
    }

    /**
     * Gets the velocity of the encoder with the conversion factor applied.
     * 
     * @return The velocity of the encoder.
     */
    @Override
    public double get() {
        if (m_device != null) {
            return m_velocity.get();
        } else {
            return super.getEncoder().getVelocity();
        }
    }

    @Override
    public void setVoltage(double voltage) {
        if (m_device != null) {
            m_voltage.set(voltage);
        } else {
            super.setVoltage(voltage);
        }
    }

    /**
     * Set the conversion factor for velocity of the encoder. Multiplied by the
     * native output units to
     * give you velocity
     *
     * @param factor The conversion factor to multiply the native units by
     * @return {@link REVLibError#kOk} if successful
     */
    public void setVelocityConversionFactor(double factor) {
        if (m_device == null) {
            super.getEncoder().setVelocityConversionFactor(factor);
        }
    }

    /**
     * Set the conversion factor for position of the encoder. Multiplied by the
     * native output units to
     * give you position.
     *
     * @param factor The conversion factor to multiply the native units by
     * @return {@link REVLibError#kOk} if successful
     */
    public void setPositionConversionFactor(double factor) {
        if (m_device == null) {
            super.getEncoder().setPositionConversionFactor(factor);
        }
    }

    /**
     * Sets the position of the simulated motor.
     * 
     * @param pos
     */
    public void setSimPosition(double pos) {
        if (m_device != null) {
            m_position.set(pos);
        }
    }

    /**
     * Gets the position of the simulated motor.
     * 
     * @return
     */
    public double getPosition() {
        if (m_device != null) {
            return m_position.get();
        } else {
            return super.getEncoder().getPosition();
        }
    }
}
