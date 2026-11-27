package org.firstinspires.ftc.teamcode;

/** Generated from visualizations/working/main-bezier.pp.json.
 * Run python3 scripts/import_main_auto_curves.py after editing control points.
 * Other motion/heading/timing edits require reviewing the mechanism sequence.
 */
final class MainAutoCurves {
    static final String GEOMETRY_SHA256 = "36382467c5c89cb9f83afc6e4aeaa035ab10eaee6a5c3a45ee644a6f54bf74d5";
    static final double FIELD_START_X = 55.9219;
    static final double FIELD_START_Y = 7.7305;
    static final double FIELD_START_HEADING_DEGREES = 270;

    static final AutoBezierCurve FIRST_INTAKE = new AutoBezierCurve(
            0.0, 0.0,
            -11.034242, -13.899101,
            -3.930814, -31.868294,
            -9.7, -47.38,
            0.0, 2.7);

    static final AutoBezierCurve SECOND_INTAKE = new AutoBezierCurve(
            -2.99041, -41.056916,
            -43.940041193538, -26.124285854602,
            -83.682412605255, -40.463746062174,
            -121.07, -15.38,
            -170, -170);

    private MainAutoCurves() { }
}
