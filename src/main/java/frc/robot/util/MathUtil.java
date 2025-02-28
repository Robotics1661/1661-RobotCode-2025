package frc.robot.util;

import edu.wpi.first.units.measure.Angle;

public class MathUtil {
    public static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
    }

    public static Angle clamp(Angle v, Angle min, Angle max) {
        if (v.lt(min)) {
            return min;
        } else if (v.gt(max)) {
            return max;
        } else {
            return v;
        }
    }

    public static double wrap_0_1(double v) {
        while (v < 0.0) {
            v += 1.0;
        }
        while (v > 1.0) {
            v -= 1.0;
        }
        return v;
    }

    /**
     * Remap {@param v} from {@param min0} - {@param max0} to {@param min1} - {@param max1}
     * @param v value
     * @param min0 lower end of input range
     * @param max0 higher end of input range
     * @param min1 lower end of output range
     * @param max1 higher end of output range
     * @return remapped value
     */
    public static double remap(double v, double min0, double max0, double min1, double max1) {
        double factor = (v - min0) / (max0 - min0);
        return (max1 - min1) * factor + min1;
    }
}
