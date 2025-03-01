package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.MathUtil.wrap_n180_p180;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class FieldPositioningUtils {
    // origin is far-right of blue alliance side (outside the field b/c angled source)

    public static final Distance fieldX = Meters.of(17.55);
    public static final Distance fieldY = Meters.of(8.05);

    public static final Distance reefInnerRadius = Inches.of((93.50 / 2.0) - 14.0);

    public static final Distance robotInnerRadius = Inches.of(34.5 / 2.);

    public static final Translation2d blueReefCenter = new Translation2d(
        Inches.of(144.0).plus(reefInnerRadius),
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

    // these are named from a driver's perspective
    public static enum ReefHextant {
        NEAR(0),
        NEAR_LEFT(-60),
        NEAR_RIGHT(60),
        FAR(180),
        FAR_LEFT(-120),
        FAR_RIGHT(120)
        ;

        /** the degrees a blue robot has to face in to point at this side of the reef */
        public final double blueFacingDeg;
        /** the degrees a red robot has to face in to point at this side of the reef */
        public final double redFacingDeg;

        ReefHextant(double blueFacingDeg) {
            this.blueFacingDeg = blueFacingDeg;
            this.redFacingDeg = wrap_n180_p180(blueFacingDeg + 180);
        }

        public static ReefHextant fromTranslation(Translation2d pos, boolean isRed) {
            return fromAngle(angleToReef(pos, isRed), isRed);
        }

        private static ReefHextant fromAngle(Angle angle, boolean isRed) {
            double angleDeg = wrap_n180_p180(angle.in(Units.Degrees) + (isRed ? 180 : 0));

            if (angleDeg >= -30 && angleDeg < 30) {
                return ReefHextant.NEAR;
            } else if (angleDeg >= 30 && angleDeg < 90) {
                return ReefHextant.NEAR_RIGHT;
            } else if (angleDeg >= 90 && angleDeg < 150) {
                return ReefHextant.FAR_RIGHT;
            } else if (angleDeg >= -90 && angleDeg < -30) {
                return ReefHextant.NEAR_LEFT;
            } else if (angleDeg >= -150 && angleDeg < -90) {
                return ReefHextant.FAR_LEFT;
            } else {
                return ReefHextant.FAR;
            }
        }

        public Rotation2d facing(boolean isRed) {
            return Rotation2d.fromDegrees(isRed ? redFacingDeg : blueFacingDeg);
        }

        public Pose2d makePose(Distance radius, boolean isRed) {
            return makePose(radius, Units.Meters.zero(), isRed);
        }

        public Pose2d makePose(Distance radius, Distance perpendicular, boolean isRed) {
            final double facingRad = Math.toRadians(isRed ? redFacingDeg : blueFacingDeg);
            final double offsetRad = facingRad + Math.PI;
            final double perpendicularRad = offsetRad + Math.PI/2.;
            final Translation2d center = isRed ? redReefCenter : blueReefCenter;

            final Translation2d radiusOffset = new Translation2d(
                radius.times(Math.cos(offsetRad)),
                radius.times(Math.sin(offsetRad))
            );

            final Translation2d perpendicularOffset = new Translation2d(
                perpendicular.times(Math.cos(perpendicularRad)),
                perpendicular.times(Math.sin(perpendicularRad))
            );

            return new Pose2d(
                center.plus(radiusOffset).plus(perpendicularOffset),
                new Rotation2d(facingRad)
            );
        }
    }
}
