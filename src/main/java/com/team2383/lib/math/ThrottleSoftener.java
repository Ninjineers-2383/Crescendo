package com.team2383.lib.math;

public class ThrottleSoftener {
    /**
     * Softens the input throttle value
     * <p>
     * soften(0.0) = 0.0
     * <p>
     * soften(1.0) = 1.0
     * <p>
     * soften(-1.0) = -1.0
     * <p>
     * https://www.desmos.com/calculator/f4z37ovuvi
     * 
     * @param input the input values [-1, 1]
     * @return the output value [-1, 1]
     */
    public static double soften(double input) {
        double gain = 0.5;
        double result = gain * (input * input * input) + (1 - gain) * input;
        return result;
    }
}
