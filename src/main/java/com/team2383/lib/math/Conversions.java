package com.team2383.lib.math;

public class Conversions {
    /**
     * @param positionCounts
     *            CANCoder Position Counts
     * @param gearRatio
     *            Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees
     *            Degrees of rotation of Mechanism
     * @param gearRatio
     *            Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param counts
     *            Falcon Position Counts
     * @param gearRatio
     *            Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param rotationCount
     *            Falcon rotor position
     * @param gearRatio
     *            Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double rotationsToDegrees(double rotationCount, double gearRatio) {
        return rotationCount * 360 / gearRatio;
    }

    /**
     * @param degrees
     *            Degrees of rotation of Mechanism
     * @param gearRatio
     *            Gear Ratio between Falcon and Mechanism
     * @return Falcon Position Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    public static double degreesToRotations(double degrees, double gearRatio) {
        return degrees / (360.0 / gearRatio);
    }

    /**
     * @param velocityCounts
     *            Falcon Velocity Counts
     * @param gearRatio
     *            Gear Ratio between Falcon and Mechanism (set to 1 for
     *            Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param falconRPS
     *            Falcon RPS
     * @param gearRatio
     *            Gear Ratio between Falcon and Mechanism (set to 1 for
     *            Falcon RPM)
     * @return RPS of Mechanism
     */
    public static double falconRPSToWheelRPS(double falconRPS, double gearRatio) {
        double mechRPS = falconRPS / gearRatio;
        return mechRPS;
    }

    /**
     * @param RPM
     *            RPM of mechanism
     * @param gearRatio
     *            Gear Ratio between Falcon and Mechanism (set to 1 for Falcon
     *            RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts
     *            Falcon Velocity Counts
     * @param circumference
     *            Circumference of Wheel
     * @param gearRatio
     *            Gear Ratio between Falcon and Mechanism (set to 1 for
     *            Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60.0;
        return wheelMPS;
    }

    /**
     * @param RPS
     *            Falcon RPS
     * @param circumference
     *            Circumference of Wheel
     * @param gearRatio
     *            Gear Ratio between Falcon and Mechanism (set to 1 for
     *            Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double RPSToMPS(double RPS, double circumference, double gearRatio) {
        double wheelRPS = falconRPSToWheelRPS(RPS, gearRatio);
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param velocity
     *            Velocity MPS
     * @param circumference
     *            Circumference of Wheel
     * @param gearRatio
     *            Gear Ratio between Falcon and Mechanism (set to 1 for
     *            Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    public static double MPSToFalconRPS(double velocity, double circumference, double gearRatio) {
        double wheelRPS = velocity / circumference;
        double rotorVelocity = wheelRPS * gearRatio;
        return rotorVelocity;
    }

    /**
     * @param positionCounts
     *            Falcon Position Counts
     * @param circumference
     *            Circumference of Wheel
     * @param gearRatio
     *            Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    public static double falconToMeters(double positionCounts, double circumference, double gearRatio) {
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }

    /**
     * @param rotations
     *            Rotation Count
     * @param circumference
     *            Circumference of Wheel
     * @param gearRatio
     *            Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    public static double rotationsToMeters(double rotations, double circumference, double gearRatio) {
        return rotations * (circumference / gearRatio);
    }

    /**
     * @param meters
     *            Meters
     * @param circumference
     *            Circumference of Wheel
     * @param gearRatio
     *            Gear Ratio between Falcon and Wheel
     * @return Falcon Position Counts
     */
    public static double MetersToFalcon(double meters, double circumference, double gearRatio) {
        return meters / (circumference / (gearRatio * 2048.0));
    }
}