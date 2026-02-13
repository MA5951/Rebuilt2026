
package frc.robot.Util;

public class InterpolationTable {

    private double[][] table;
    private int DIST = 0, ANGLE = 1;
    private int closestIdx = 0;
    private double minAbs;
    private double[] closest;
    private double[] other;
    private double[] a;
    private double[] b;
    private double x0, x1, t, y0, y1;

    public InterpolationTable(double[][] table) {
        this.table = table;

    }

    public double interpolate(double distance) {

        // Find closest point index
        minAbs = Double.MAX_VALUE;
        for (int i = 0; i < table.length; i++) {
            double abs = Math.abs(distance - table[i][DIST]);
            if (abs < minAbs) {
                minAbs = abs;
                closestIdx = i;
            }
        }

        closest = table[closestIdx];

        // If outside bounds -> clamp to closest endpoint
        if ((closestIdx == 0 && distance < closest[DIST]) ||
                (closestIdx == table.length - 1 && distance > closest[DIST])) {
            return closest[ANGLE];
        }

        // Pick neighbor on the correct side
        other = (distance > closest[DIST]) ? table[closestIdx + 1] : table[closestIdx - 1];

        // Order so (x0 <= x1)
        a = (other[DIST] < closest[DIST]) ? other : closest; // smaller distance
        b = (other[DIST] < closest[DIST]) ? closest : other; // bigger distance

        // Linear interpolation
        x0 = a[DIST];
        y0 = a[ANGLE];
        x1 = b[DIST];
        y1 = b[ANGLE];

        if (x1 == x0)
            return y0; // safety

        t = (distance - x0) / (x1 - x0);
        return y0 + (y1 - y0) * t;
    }

}
