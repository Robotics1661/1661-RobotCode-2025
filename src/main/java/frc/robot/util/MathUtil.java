package frc.robot.util;

public class MathUtil {
    public static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
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
