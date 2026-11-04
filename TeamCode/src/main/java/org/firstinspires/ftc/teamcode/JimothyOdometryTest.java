package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

/** Push-only test for checking three-wheel odometry signs and calibration. */
@TeleOp(name = "Jimothy Odometry Test", group = "Jimothy Tests")
public class JimothyOdometryTest extends LinearOpMode {
    private static final String TAG = "JimothyOdoTest";
    private static final double LEFT_TICKS_PER_INCH = 509.66;
    private static final double RIGHT_TICKS_PER_INCH = 507.60;
    private static final double CENTER_TICKS_PER_INCH = 507.92;
    private static final double POD_SPACING_INCHES = 12.263;
    private static final double CENTER_POD_X_INCHES = -6.718;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor leftPod;
    private DcMotor rightPod;
    private DcMotor centerPod;

    private int previousLeft;
    private int previousRight;
    private int previousCenter;
    private int originLeft;
    private int originRight;
    private int originCenter;
    private double x;
    private double y;
    private double heading;
    private final com.qualcomm.robotcore.util.ElapsedTime pathTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();
    private final com.qualcomm.robotcore.util.ElapsedTime pathLogTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftPod = frontLeft;
        rightPod = hardwareMap.get(DcMotor.class, "odo_right");
        centerPod = hardwareMap.get(DcMotor.class, "odo_center");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        makeDriveSafe(frontLeft);
        makeDriveSafe(frontRight);
        makeDriveSafe(backLeft);
        makeDriveSafe(backRight);
        preparePod(rightPod);
        preparePod(centerPod);
        resetPose();

        telemetry.addLine("INTAKE PATH TEST: drivetrain power stays zero");
        telemetry.addLine("Place robot at shooting position before Play");
        telemetry.addLine("A = reset origin, B = log checkpoint");
        telemetry.addLine("Expected: forward +X, left +Y, CCW +heading");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        resetPose();
        pathTimer.reset();
        pathLogTimer.reset();
        RobotLog.ii(TAG, "INTAKE_PATH_START pose=(0.00,0.00,0.0)");

        boolean previousA = false;
        boolean previousB = false;
        boolean recordingComplete = false;
        while (opModeIsActive()) {
            if (gamepad1.a && !previousA) {
                resetPose();
                pathTimer.reset();
                recordingComplete = false;
                RobotLog.ii(TAG, "ORIGIN_RESET");
            }
            updatePose();
            if (!recordingComplete && pathLogTimer.milliseconds() >= 100) {
                RobotLog.ii(TAG,
                        "INTAKE_PATH time=%.2f x=%.2f y=%.2f heading=%.1f",
                        pathTimer.seconds(), x, y, Math.toDegrees(heading));
                pathLogTimer.reset();
            }
            if (gamepad1.b && !previousB) {
                RobotLog.ii(TAG,
                        "INTAKE_POSE time=%.2f raw=(%d,%d,%d) delta=(%d,%d,%d) pose=(%.2f,%.2f,%.1f)",
                        pathTimer.seconds(),
                        readLeft(), readRight(), readCenter(),
                        readLeft() - originLeft, readRight() - originRight,
                        readCenter() - originCenter, x, y, Math.toDegrees(heading));
                recordingComplete = true;
            }
            previousA = gamepad1.a;
            previousB = gamepad1.b;

            telemetry.addLine("Push along desired route; keep final heading near 0");
            telemetry.addData("Pose X forward", "%+.2f in", x);
            telemetry.addData("Pose Y left", "%+.2f in", y);
            telemetry.addData("Heading CCW", "%+.1f deg", Math.toDegrees(heading));
            telemetry.addData("Left raw / delta", "%d / %d", readLeft(),
                    readLeft() - originLeft);
            telemetry.addData("Right raw / delta", "%d / %d", readRight(),
                    readRight() - originRight);
            telemetry.addData("Center raw / delta", "%d / %d", readCenter(),
                    readCenter() - originCenter);
            telemetry.addData("Recorder", recordingComplete
                    ? "CAPTURED - logging frozen" : "RECORDING PATH");
            telemetry.addLine("A reset/new recording | B capture and freeze | STOP when done");
            telemetry.update();
            idle();
        }
    }

    private void updatePose() {
        int currentLeft = readLeft();
        int currentRight = readRight();
        int currentCenter = readCenter();
        double deltaLeft = (currentLeft - previousLeft) / LEFT_TICKS_PER_INCH;
        double deltaRight = (currentRight - previousRight) / RIGHT_TICKS_PER_INCH;
        double deltaCenter = (currentCenter - previousCenter) / CENTER_TICKS_PER_INCH;
        previousLeft = currentLeft;
        previousRight = currentRight;
        previousCenter = currentCenter;

        // With the installed pod directions, left-minus-right is positive for
        // a counterclockwise turn.
        double deltaHeading = (deltaLeft - deltaRight) / POD_SPACING_INCHES;
        double forward = (deltaLeft + deltaRight) / 2.0;
        double lateral = deltaCenter - CENTER_POD_X_INCHES * deltaHeading;
        double midpoint = heading + deltaHeading / 2.0;
        x += forward * Math.cos(midpoint) - lateral * Math.sin(midpoint);
        y += forward * Math.sin(midpoint) + lateral * Math.cos(midpoint);
        heading = normalizeRadians(heading + deltaHeading);
    }

    private void resetPose() {
        previousLeft = originLeft = readLeft();
        previousRight = originRight = readRight();
        previousCenter = originCenter = readCenter();
        x = 0;
        y = 0;
        heading = 0;
    }

    private void makeDriveSafe(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);
    }

    private void preparePod(DcMotor pod) {
        pod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pod.setPower(0);
    }

    private int readLeft() {
        return leftPod.getCurrentPosition();
    }

    private int readRight() {
        return rightPod.getCurrentPosition();
    }

    private int readCenter() {
        return -centerPod.getCurrentPosition();
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
