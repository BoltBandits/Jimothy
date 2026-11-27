package org.firstinspires.ftc.teamcode;

/** Cubic geometry in the autonomous odometry frame (inches, radians). */
final class AutoBezierCurve {
    private static final int SAMPLES = 256;
    private final double[] controlX = new double[4];
    private final double[] controlY = new double[4];
    private final double[] sampleX = new double[SAMPLES + 1];
    private final double[] sampleY = new double[SAMPLES + 1];
    private final double[] distance = new double[SAMPLES + 1];
    private final double startHeading;
    private final double headingChange;

    AutoBezierCurve(double startX, double startY, double cp1X, double cp1Y,
                    double cp2X, double cp2Y, double endX, double endY,
                    double startHeadingDegrees, double endHeadingDegrees) {
        double[] xs = {startX, cp1X, cp2X, endX};
        double[] ys = {startY, cp1Y, cp2Y, endY};
        System.arraycopy(xs, 0, controlX, 0, 4);
        System.arraycopy(ys, 0, controlY, 0, 4);
        startHeading = Math.toRadians(startHeadingDegrees);
        headingChange = Math.atan2(
                Math.sin(Math.toRadians(endHeadingDegrees - startHeadingDegrees)),
                Math.cos(Math.toRadians(endHeadingDegrees - startHeadingDegrees)));
        for (int i = 0; i <= SAMPLES; i++) {
            double t = (double) i / SAMPLES;
            sampleX[i] = coordinate(controlX, t);
            sampleY[i] = coordinate(controlY, t);
            if (i > 0) {
                distance[i] = distance[i - 1] + Math.hypot(
                        sampleX[i] - sampleX[i - 1], sampleY[i] - sampleY[i - 1]);
            }
        }
    }

    double length() { return distance[SAMPLES]; }
    double endX() { return controlX[3]; }
    double endY() { return controlY[3]; }

    /** Project onto the sampled curve, returning distance along it in inches. */
    double closestDistance(double x, double y) {
        double bestSquared = Double.POSITIVE_INFINITY;
        double bestDistance = 0.0;
        for (int i = 1; i <= SAMPLES; i++) {
            double dx = sampleX[i] - sampleX[i - 1];
            double dy = sampleY[i] - sampleY[i - 1];
            double squaredLength = dx * dx + dy * dy;
            double fraction = squaredLength < 1e-12 ? 0.0 : clip(
                    ((x - sampleX[i - 1]) * dx + (y - sampleY[i - 1]) * dy)
                            / squaredLength, 0.0, 1.0);
            double errorX = x - sampleX[i - 1] - fraction * dx;
            double errorY = y - sampleY[i - 1] - fraction * dy;
            double squaredError = errorX * errorX + errorY * errorY;
            if (squaredError < bestSquared) {
                bestSquared = squaredError;
                bestDistance = distance[i - 1]
                        + fraction * (distance[i] - distance[i - 1]);
            }
        }
        return bestDistance;
    }

    double xAtDistance(double inches) {
        return coordinate(controlX, parameterAtDistance(inches));
    }

    double yAtDistance(double inches) {
        return coordinate(controlY, parameterAtDistance(inches));
    }

    /** Match the visualizer's linear heading interpolation by arc length. */
    double headingAtDistance(double inches) {
        double fraction = length() < 1e-9 ? 1.0 : clip(inches / length(), 0.0, 1.0);
        return startHeading + fraction * headingChange;
    }

    private double parameterAtDistance(double inches) {
        if (inches <= 0.0) return 0.0;
        if (inches >= length()) return 1.0;
        int low = 0;
        int high = SAMPLES;
        while (high - low > 1) {
            int mid = (low + high) / 2;
            if (distance[mid] < inches) low = mid;
            else high = mid;
        }
        double span = distance[high] - distance[low];
        double fraction = span < 1e-12 ? 0.0 : (inches - distance[low]) / span;
        return (low + fraction) / SAMPLES;
    }

    private static double coordinate(double[] points, double t) {
        double u = 1.0 - t;
        return u * u * u * points[0] + 3.0 * u * u * t * points[1]
                + 3.0 * u * t * t * points[2] + t * t * t * points[3];
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
