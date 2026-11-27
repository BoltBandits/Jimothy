package org.firstinspires.ftc.teamcode;

/** Hardware-independent numerical checks; run by test_main_auto_curves.py. */
public final class AutoBezierCurveChecks {
    private static void near(double actual, double expected, double tolerance) {
        if (!Double.isFinite(actual) || Math.abs(actual - expected) > tolerance) {
            throw new AssertionError(actual + " != " + expected);
        }
    }

    private static String samples(AutoBezierCurve curve) {
        StringBuilder out = new StringBuilder("{\"length\":" + curve.length() + ",\"points\":[");
        for (int i = 0; i <= 20; i++) {
            double distance = curve.length() * i / 20.0;
            double x = curve.xAtDistance(distance);
            double y = curve.yAtDistance(distance);
            // Projection must recover distance along both real, edited curves.
            near(curve.closestDistance(x, y), distance, 0.002);
            if (i > 0) out.append(',');
            out.append('[').append(x).append(',').append(y).append(',')
                    .append(curve.headingAtDistance(distance)).append(']');
        }
        return out.append("]}").toString();
    }

    public static void main(String[] args) {
        AutoBezierCurve arch = new AutoBezierCurve(0, 0, 0, 10, 10, 10, 10, 0, 350, 10);
        near(arch.length(), 20, 0.001);
        near(arch.xAtDistance(arch.length() / 2), 5, 1e-6);
        near(arch.yAtDistance(arch.length() / 2), 7.5, 1e-6);
        near(arch.headingAtDistance(arch.length() / 2), Math.toRadians(360), 1e-9);
        near(arch.closestDistance(5, 9), arch.length() / 2, 0.001);

        // Nonuniform curve parameterization must still advance by distance.
        AutoBezierCurve line = new AutoBezierCurve(0, 0, 0, 0, 0, 0, 10, 0, 0, 90);
        near(line.xAtDistance(5), 5, 0.001);
        near(line.headingAtDistance(5), Math.PI / 4, 1e-9);
        near(line.closestDistance(-4, 0), 0, 1e-9);
        near(line.closestDistance(15, 0), 10, 1e-9);
        near(line.xAtDistance(-1), 0, 1e-9);
        near(line.xAtDistance(12), 10, 1e-9);

        AutoBezierCurve point = new AutoBezierCurve(3, 4, 3, 4, 3, 4, 3, 4, 0, 90);
        near(point.xAtDistance(0), 3, 1e-9);
        near(point.closestDistance(3, 4), 0, 1e-9);
        near(point.headingAtDistance(0), Math.PI / 2, 1e-9);

        System.out.println("{\"first-intake-bezier\":" + samples(MainAutoCurves.FIRST_INTAKE)
                + ",\"second-intake-bezier\":" + samples(MainAutoCurves.SECOND_INTAKE) + "}");
    }
}
