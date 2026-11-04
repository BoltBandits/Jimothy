package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "Jimothy Auto Park", group = "Jimothy")
public class JimothyAutoPark extends LinearOpMode {
    private static final String TAG = "JimothyAutoPark";

    private static final double CORNER_X = 80.46;
    private static final double CORNER_Y = -0.21;
    private static final double CORNER_HEADING = -4.8;
    private static final double PARK_X = 83.20;
    private static final double PARK_Y = 19.25;
    private static final double PARK_HEADING = -4.1;

    private static final double LEFT_TICKS_PER_INCH = 509.66;
    private static final double RIGHT_TICKS_PER_INCH = 507.60;
    private static final double CENTER_TICKS_PER_INCH = 507.92;
    private static final double POD_SPACING_INCHES = 12.263;
    private static final double CENTER_POD_X_INCHES = -6.718;
    private static final double TRANSLATION_KP = 0.075;
    private static final double HEADING_KP = 0.90;
    private static final double MIN_FORWARD = 0.10;
    private static final double MIN_STRAFE = 0.16;
    private static final double MIN_TURN = 0.19;
    private static final double MAX_TURN = 0.70;
    private static final double DRIVE_TIMEOUT_SECONDS = 22.0;

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

    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.addLine("JIMOTHY AUTO PARK READY");
        telemetry.addData("Parking pose", "X %.2f Y %.2f H %.1f",
                PARK_X, PARK_Y, PARK_HEADING);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        resetPose();

        try {
            if (!driveToPose(CORNER_X, CORNER_Y, CORNER_HEADING,
                    2.0, Math.toRadians(5.0), true,
                    "Parking path 1/2: forward")) return;
            if (!driveToPose(PARK_X, PARK_Y, PARK_HEADING,
                    1.25, Math.toRadians(3.0), true,
                    "Parking path 2/2: final")) return;

            while (opModeIsActive()) {
                stopDrive();
                telemetry.addData("Step", "PARKED");
                telemetry.addData("Pose", "X %+.2f Y %+.2f H %+.1f",
                        x, y, Math.toDegrees(heading));
                telemetry.update();
                idle();
            }
        } finally {
            stopDrive();
        }
    }

    private void initializeHardware() {
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
        prepareDrive(frontLeft);
        prepareDrive(frontRight);
        prepareDrive(backLeft);
        prepareDrive(backRight);
        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetPose();
    }

    private void prepareDrive(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

    private boolean driveToPose(double targetX, double targetY,
                                double targetHeadingDegrees,
                                double positionTolerance,
                                double headingTolerance,
                                boolean stopAtTarget, String step) {
        ElapsedTime timer = new ElapsedTime();
        double targetHeading = Math.toRadians(targetHeadingDegrees);
        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {
            updatePose();
            double errorX = targetX - x;
            double errorY = targetY - y;
            double positionError = Math.hypot(errorX, errorY);
            double headingError = normalize(targetHeading - heading);
            if (positionError <= positionTolerance
                    && Math.abs(headingError) <= headingTolerance) {
                if (stopAtTarget) stopDrive();
                RobotLog.ii(TAG, "WAYPOINT_REACHED step=%s pose=(%.2f,%.2f,%.1f)",
                        step, x, y, Math.toDegrees(heading));
                return true;
            }

            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double forwardError = errorX * cos + errorY * sin;
            double leftError = -errorX * sin + errorY * cos;
            double axisTolerance = positionTolerance / Math.sqrt(2.0);
            double forward = minimumPower(
                    Range.clip(forwardError * TRANSLATION_KP, -1.0, 1.0),
                    MIN_FORWARD, forwardError);
            double left = minimumPower(
                    Range.clip(leftError * TRANSLATION_KP, -1.0, 1.0),
                    MIN_STRAFE, leftError);
            double turn = Range.clip(headingError * HEADING_KP, -MAX_TURN, MAX_TURN);
            if (Math.abs(forwardError) <= axisTolerance) forward = 0;
            if (Math.abs(leftError) <= axisTolerance) left = 0;
            if (Math.abs(headingError) <= headingTolerance) turn = 0;
            else if (Math.abs(turn) < MIN_TURN) turn = Math.copySign(MIN_TURN, turn);
            setMecanum(forward, -left, -turn);

            telemetry.addData("Step", step);
            telemetry.addData("Target", "X %+.1f Y %+.1f H %+.1f",
                    targetX, targetY, targetHeadingDegrees);
            telemetry.addData("Pose", "X %+.1f Y %+.1f H %+.1f",
                    x, y, Math.toDegrees(heading));
            telemetry.addData("Position error", "%.1f in", positionError);
            telemetry.addData("Time", "%.1f / %.1f sec",
                    timer.seconds(), DRIVE_TIMEOUT_SECONDS);
            telemetry.update();
            idle();
        }
        stopDrive();
        if (!opModeIsActive()) return false;
        RobotLog.ww(TAG, "WAYPOINT_TIMEOUT step=%s pose=(%.2f,%.2f,%.1f)",
                step, x, y, Math.toDegrees(heading));
        return true;
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
        double deltaHeading = (deltaLeft - deltaRight) / POD_SPACING_INCHES;
        double forward = (deltaLeft + deltaRight) / 2.0;
        double lateral = deltaCenter - CENTER_POD_X_INCHES * deltaHeading;
        double midpoint = heading + deltaHeading / 2.0;
        x += forward * Math.cos(midpoint) - lateral * Math.sin(midpoint);
        y += forward * Math.sin(midpoint) + lateral * Math.cos(midpoint);
        heading = normalize(heading + deltaHeading);
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

    private double minimumPower(double power, double minimum, double error) {
        return Math.abs(power) < minimum && Math.abs(error) > 0
                ? Math.copySign(minimum, error) : power;
    }

    private void setMecanum(double forward, double left, double turn) {
        double fl = forward + left + turn;
        double fr = forward - left - turn;
        double bl = forward - left + turn;
        double br = forward + left - turn;
        double scale = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / scale);
        frontRight.setPower(fr / scale);
        backLeft.setPower(bl / scale);
        backRight.setPower(br / scale);
    }

    private void stopDrive() { setMecanum(0, 0, 0); }

    private double normalize(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
