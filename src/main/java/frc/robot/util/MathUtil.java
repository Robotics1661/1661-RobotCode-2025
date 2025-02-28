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
}
