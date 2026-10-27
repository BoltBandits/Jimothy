package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

/** Drives from the autonomous start and captures a three-pod field pose for parking. */
@TeleOp(name = "Jimothy Park Pose Capture", group = "Diagnostics")
public class JimothyParkPoseCapture extends LinearOpMode {
    private static final String TAG = "ParkPoseCapture";
    private static final double LEFT_TICKS_PER_INCH = 509.66;
    private static final double RIGHT_TICKS_PER_INCH = 507.60;
    private static final double CENTER_TICKS_PER_INCH = 507.92;
    private static final double POD_SPACING_INCHES = 12.263;
    private static final double CENTER_POD_X_INCHES = -6.718;
    private static final double DRIVE_SCALE = 0.55;
    private static final double SLOW_SCALE = 0.25;

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
    private double x;
    private double y;
    private double heading;
    private boolean captured;
    private boolean previousA;
    private boolean previousB;
    private boolean previousReset;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");
        // After the hardware change, Control Hub encoder port 0 reads the left pod.
        leftPod = frontLeft;
        rightPod = hardwareMap.get(DcMotor.class, "odo_right");
        centerPod = hardwareMap.get(DcMotor.class, "odo_center");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        prepareDrive(frontLeft);
        prepareDrive(frontRight);
        prepareDrive(backLeft);
        prepareDrive(backRight);
        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetPose();

        telemetry.addLine("Place robot at the normal autonomous start");
        telemetry.addLine("Controller 1: left stick drive, right stick X turn");
        telemetry.addLine("LB slow | A capture/freeze | B resume | Options reset pose");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        resetPose();

        try {
            while (opModeIsActive()) {
                updatePose();

                boolean a = gamepad1.a;
                if (a && !previousA) {
                    captured = true;
                    stopDrive();
                    RobotLog.ii(TAG, "PARK_POSE_CAPTURED x=%.2f y=%.2f heading=%.1f",
                            x, y, Math.toDegrees(heading));
                }
                previousA = a;

                boolean b = gamepad1.b;
                if (b && !previousB) captured = false;
                previousB = b;

                boolean reset = gamepad1.options || gamepad1.start;
                if (reset && !previousReset) {
                    resetPose();
                    captured = false;
                }
                previousReset = reset;

                if (!captured) {
                    double scale = gamepad1.left_bumper ? SLOW_SCALE : DRIVE_SCALE;
                    double forward = -gamepad1.left_stick_y * scale;
                    double left = -gamepad1.left_stick_x * scale;
                    double ccw = gamepad1.right_stick_x * scale;
                    // Match the autonomous drivetrain mounting convention.
                    setMecanum(forward, -left, -ccw);
                } else {
                    stopDrive();
                }

                telemetry.addData("Status", captured ? "CAPTURED - send these values" : "DRIVING");
                telemetry.addData("X", "%+.2f in", x);
                telemetry.addData("Y", "%+.2f in", y);
                telemetry.addData("Heading", "%+.1f deg", Math.toDegrees(heading));
                telemetry.addData("Pod ticks L / R / C", "%d / %d / %d",
                        readLeft(), readRight(), readCenter());
                telemetry.update();
                idle();
            }
        } finally {
            stopDrive();
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

        double deltaHeading = (deltaRight - deltaLeft) / POD_SPACING_INCHES;
        double forward = (deltaLeft + deltaRight) / 2.0;
        double lateral = deltaCenter - CENTER_POD_X_INCHES * deltaHeading;
        double midpoint = heading + deltaHeading / 2.0;
        x += forward * Math.cos(midpoint) - lateral * Math.sin(midpoint);
        y += forward * Math.sin(midpoint) + lateral * Math.cos(midpoint);
        heading = normalizeRadians(heading + deltaHeading);
    }

    private void resetPose() {
        previousLeft = readLeft();
        previousRight = readRight();
        previousCenter = readCenter();
        x = 0;
        y = 0;
        heading = 0;
    }

    private int readLeft() { return leftPod.getCurrentPosition(); }
    private int readRight() { return rightPod.getCurrentPosition(); }
    private int readCenter() { return -centerPod.getCurrentPosition(); }

    private void prepareDrive(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

    private void setMecanum(double forward, double left, double turn) {
        double fl = forward + left + turn;
        double fr = forward - left - turn;
        double bl = forward - left + turn;
        double br = forward + left - turn;
        double scale = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(Range.clip(fl / scale, -1, 1));
        frontRight.setPower(Range.clip(fr / scale, -1, 1));
        backLeft.setPower(Range.clip(bl / scale, -1, 1));
        backRight.setPower(Range.clip(br / scale, -1, 1));
    }

    private void stopDrive() { setMecanum(0, 0, 0); }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
