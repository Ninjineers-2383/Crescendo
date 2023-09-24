package com.team2383.robot.helpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleOptimizer {
    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially reversing the direction the wheel spins. If this is used with
     * the PIDController class's continuous input functionality, the furthest a
     * wheel will ever rotate is 90 degrees. This also allows you to pass in an
     * angle for the module to attain when static.
     *
     * @param desiredState
     *            The desired state.
     * @param currentAngle
     *            The current module angle.
     * @param staticAngle
     *            The angle the module should attain when static.
     * @return Optimized swerve module state.
     */
    public static SwerveModuleState customOptimize(
            SwerveModuleState desiredState, Rotation2d currentAngle,
            Rotation2d staticAngle) {

        // if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
        // desiredState = new SwerveModuleState(0, staticAngle);
        // }
        return SwerveModuleState.optimize(desiredState, currentAngle);
    }
}
