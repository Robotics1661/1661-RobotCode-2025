package frc.robot.util;

public class MathUtil {
    public static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
    }
}
