package com.team2383.lib.math;

import edu.wpi.first.math.MathUtil;

public class Clip {
    /**
     * Clips the value to inside the limits
     * 
     * @param min   minimum value
     * @param value input value
     * @param max   max value
     * @return the value or the limits
     */
    public static double clip(double min, double value, double max) {
        return MathUtil.clamp(value, min, max);
    }

    /**
     * Keeps the value outside the limits
     * 
     * @param min   minimum value
     * @param value input value
     * @param max   max value
     * @return the value or the limits
     */
    public static double invClip(double min, double value, double max) {
        return ((value > max) ? value : ((value < min) ? value : min));
    }
}
