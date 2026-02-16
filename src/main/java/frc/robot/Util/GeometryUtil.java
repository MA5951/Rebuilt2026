
package frc.robot.Util;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeometryUtil {

    public static boolean willShotHitNet(
            Pose2d robotPoseField,
            Translation2d shooterOffsetRobot,
            Rotation2d shooterYawRobot,
            Translation2d netA,
            Translation2d netB,
            double maxRangeMeters,
            double hitToleranceMeters) {
        // 1) Shooter exit point in FIELD coordinates
        Translation2d shooterPosField = robotPoseField.getTranslation()
                .plus(shooterOffsetRobot.rotateBy(robotPoseField.getRotation()));

        // 2) Shot direction in FIELD coordinates
        Rotation2d shotHeadingField = robotPoseField.getRotation().plus(shooterYawRobot);
        Translation2d dir = new Translation2d(shotHeadingField.getCos(), shotHeadingField.getSin());

        // Ray: P(t) = shooterPosField + dir * t, t >= 0
        // Segment: S(u) = netA + (netB - netA) * u, 0 <= u <= 1

        Translation2d v = netB.minus(netA); // segment direction

        double denom = cross(dir, v);

        // If denom ~ 0, ray and segment are parallel. Use distance-to-segment check.
        if (Math.abs(denom) < 1e-9) {
            // If also collinear-ish, just check minimum distance from shooter ray line to
            // segment.
            double dist = distanceRayToSegment(shooterPosField, dir, netA, netB, maxRangeMeters);
            return dist <= hitToleranceMeters;
        }

        // Solve for t and u using 2D cross products:
        // t = cross((netA - shooterPosField), v) / cross(dir, v)
        // u = cross((netA - shooterPosField), dir) / cross(dir, v)
        Translation2d aMinusP = netA.minus(shooterPosField);
        double t = cross(aMinusP, v) / denom;
        double u = cross(aMinusP, dir) / denom;

        // Intersection must be forward along the ray and within range, and within the
        // segment
        if (t < 0.0 || t > maxRangeMeters)
            return false;
        if (u < 0.0 || u > 1.0)
            return false;

        // Exact intersection point hits the segment line.
        // If you want thickness, also accept near-misses:
        Translation2d intersection = shooterPosField.plus(dir.times(t));
        double distToSegment = distancePointToSegment(intersection, netA, netB);
        return distToSegment <= hitToleranceMeters;
    }

    // 2D cross product (a.x*b.y - a.y*b.x)
    private static double cross(Translation2d a, Translation2d b) {
        return a.getX() * b.getY() - a.getY() * b.getX();
    }

    private static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    /**
     * Distance from a point P to a segment AB.
     */
    private static double distancePointToSegment(Translation2d p, Translation2d a, Translation2d b) {
        Translation2d ab = b.minus(a);
        double ab2 = dot(ab, ab);
        if (ab2 < 1e-12)
            return p.getDistance(a); // a==b

        double t = dot(p.minus(a), ab) / ab2;
        t = clamp(t, 0.0, 1.0);
        Translation2d proj = a.plus(ab.times(t));
        return p.getDistance(proj);
    }

    /**
     * Distance from a finite ray segment (P -> P + dir*maxRange) to segment AB.
     * We approximate by checking the closest point between the segments via
     * sampling endpoints + projection.
     * (Good enough for "hit the net?" with tolerance.)
     */
    private static double distanceRayToSegment(
            Translation2d p, Translation2d dir, Translation2d a, Translation2d b, double maxRange) {
        Translation2d q = p.plus(dir.times(maxRange)); // end of ray segment
        // Minimum of: endpoints to other segment
        double d1 = distancePointToSegment(p, a, b);
        double d2 = distancePointToSegment(q, a, b);
        double d3 = distancePointToSegment(a, p, q);
        double d4 = distancePointToSegment(b, p, q);
        return Math.min(Math.min(d1, d2), Math.min(d3, d4));
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    public static double maxDistanceUntilExitField(
            Pose2d robotPoseField,
            Translation2d shooterOffsetRobot, // shooter exit offset in robot frame (meters)
            Rotation2d shooterYawRobot, // shooter yaw relative to robot forward
            double fieldLength,
            double fieldWidth) {
        // Shooter exit point in FIELD coordinates
        Translation2d shooterPosField = robotPoseField.getTranslation()
                .plus(shooterOffsetRobot.rotateBy(robotPoseField.getRotation()));

        // Shot direction in FIELD coordinates (unit vector)
        Rotation2d shotHeadingField = robotPoseField.getRotation().plus(shooterYawRobot);
        double dx = shotHeadingField.getCos();
        double dy = shotHeadingField.getSin();

        double x = shooterPosField.getX();
        double y = shooterPosField.getY();

        // Compute t to vertical boundaries (x = 0 or x = fieldLength)
        double tx = Double.POSITIVE_INFINITY;
        if (Math.abs(dx) > 1e-12) {
            double tToX0 = (0.0 - x) / dx;
            double tToXL = (fieldLength - x) / dx;
            tx = minPositive(tToX0, tToXL);
        }

        // Compute t to horizontal boundaries (y = 0 or y = fieldWidth)
        double ty = Double.POSITIVE_INFINITY;
        if (Math.abs(dy) > 1e-12) {
            double tToY0 = (0.0 - y) / dy;
            double tToYW = (fieldWidth - y) / dy;
            ty = minPositive(tToY0, tToYW);
        }

        // The first boundary you hit is the smaller positive t
        double tExit = Math.min(tx, ty);

        return tExit;
    }

    private static double minPositive(double a, double b) {
        double best = Double.POSITIVE_INFINITY;
        if (a > 1e-12)
            best = Math.min(best, a);
        if (b > 1e-12)
            best = Math.min(best, b);
        return best;
    }

    public static class Result {
        public final boolean valid; // true if the ray hits x=xLine in front of shooter
        public final double distanceMeters; // distance along shot direction (t)
        public final Translation2d hitPointField;

        public Result(boolean valid, double distanceMeters, Translation2d hitPointField) {
            this.valid = valid;
            this.distanceMeters = distanceMeters;
            this.hitPointField = hitPointField;
        }
    }



    public static Result distanceToVerticalLineX(
      Pose2d robotPoseField,
      Translation2d shooterOffsetRobot,
      Rotation2d shooterYawRobot,
      double xLine
  ) {
    // Shooter exit point in FIELD coordinates
    Translation2d p0 =
        robotPoseField.getTranslation().plus(shooterOffsetRobot.rotateBy(robotPoseField.getRotation()));

    // Shot direction in FIELD coordinates (unit vector)
    Rotation2d heading = robotPoseField.getRotation().plus(shooterYawRobot);
    double dy = heading.getCos();
    double dx = heading.getSin();

    // If dx == 0, shot is parallel to x=constant lines -> never intersects (unless already on the line).
    if (Math.abs(dx) < 1e-12) {
      // If you're exactly on the line, distance is 0.
      if (Math.abs(p0.getX() - xLine) < 1e-9) {
        return new Result(true, 0.0, p0);
      }
      return new Result(false, -1, new Translation2d(-1,-1));
    }

    // Solve for t where x(t)=xLine
    double t = (xLine - p0.getX()) / dx;

    // We only care about intersections in front of the shooter (t >= 0)
    if (t < 0.0) {
      return new Result(false, -1, new Translation2d(-1,-1));
    }

    Translation2d hit = new Translation2d(
        xLine,
        p0.getY() + dy * t
    );

    // Since direction is unit-length, distance traveled == t
    return new Result(true, t, hit);
  }


  public static Translation2d poseAdjust(
      Pose2d robotPoseField,
      Translation2d offsetRobot
  ) {
    return robotPoseField.getTranslation()
        .plus(offsetRobot.rotateBy(robotPoseField.getRotation()));
  }

  public static double angleTo(Pose2d robotPoseField, Translation2d targetField) {
    final double targetFieldAngle = Math.atan2(targetField.getY() - robotPoseField.getY(), targetField.getX() - robotPoseField.getX());
    final double robotFieldAngle  = robotPoseField.getRotation().getRadians();

    return Math.toDegrees(Math.atan2(Math.sin(targetFieldAngle - robotFieldAngle),
                      Math.cos(targetFieldAngle - robotFieldAngle)));
}


}
