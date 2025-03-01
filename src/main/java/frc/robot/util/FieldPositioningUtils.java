package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class FieldPositioningUtils {
    // origin is far-right of blue alliance side (outside the field b/c angled source)

    public static final Distance fieldX = Meters.of(17.55);
    public static final Distance fieldY = Meters.of(8.05);

    public static final Translation2d blueReefCenter = new Translation2d(
        Inches.of(144.0 + (93.50 / 2.0) - 14.0),
        fieldY.div(2.0)
    );
    public static final Translation2d redReefCenter = new Translation2d(
        fieldX.minus(blueReefCenter.getMeasureX()),
        blueReefCenter.getMeasureY()
    );

    public static Angle angleToReef(Translation2d pos, boolean isRed) {
        final Translation2d center = isRed ? redReefCenter : blueReefCenter;

        double dy = center.getY() - pos.getY();
        double dx = center.getX() - pos.getX();

        return Radians.of(Math.atan2(dy, dx));
    }
}
