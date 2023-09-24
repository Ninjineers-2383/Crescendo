package com.team2383.lib.math;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * A class to help with velocity calculations
 */
public class AngularVelocityWrapper {
    private Rotation2d lastDisplacement;
    private double lastTime;
    private Rotation2d velocity = Rotation2d.fromDegrees(0);
    private final MedianFilter filter = new MedianFilter(5);

    public AngularVelocityWrapper() {
        this(Rotation2d.fromDegrees(0));
    }

    public AngularVelocityWrapper(Rotation2d initialPosition) {
        lastDisplacement = initialPosition;
        lastTime = Timer.getFPGATimestamp();
    }

    /**
     * Calculates the velocity of a system given a displacement
     * Need to run this every loop to be accurate
     * 
     * @param displacement
     * @return discrete-time-derivative velocity
     */
    public Rotation2d calculate(Rotation2d displacement) {
        double time = Timer.getFPGATimestamp();
        velocity = Rotation2d.fromRadians(
                filter.calculate(displacement.getRadians() - lastDisplacement.getRadians()) / (time - lastTime));
        lastDisplacement = displacement;
        lastTime = time;
        return velocity;
    }

    /** Returns velocity value without recalculating */
    public Rotation2d get() {
        return velocity;
    }
}
