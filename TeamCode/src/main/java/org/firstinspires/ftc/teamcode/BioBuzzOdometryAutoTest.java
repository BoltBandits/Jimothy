package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

/** Low-speed closed-loop autonomous validation using Jimothy's calibrated odometry. */
@Autonomous(name = "BioBuzz Odometry Auto Test", group = "Diagnostics")
public class BioBuzzOdometryAutoTest extends LinearOpMode {
    private static final String TAG = "BioBuzzOdoAuto";
    private static final double LEFT_TICKS_PER_INCH = 509.66;
    private static final double RIGHT_TICKS_PER_INCH = 507.60;
    private static final double CENTER_TICKS_PER_INCH = 507.92;
    private static final double PARALLEL_POD_SPACING_INCHES = 12.263;
    private static final double CENTER_POD_X_INCHES = -6.718;

    private static final double TRANSLATION_KP = 0.045;
    private static final double HEADING_KP = 0.65;
    private static final double MAX_FORWARD_POWER = 0.24;
    // Mecanum strafing needs more authority than forward motion on this robot.
    private static final double MAX_STRAFE_POWER = 0.38;
    private static final double MAX_TURN_POWER = 0.18;
    private static final double MIN_FORWARD_POWER = 0.10;
    private static final double MIN_STRAFE_POWER = 0.16;
    private static final double AXIS_CORRECTION_DEADBAND_INCHES = 0.75;
    private static final double MIN_TURN_POWER = 0.09;
    private static final double POSITION_TOLERANCE_INCHES = 1.25;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.0);
    private static final double WAYPOINT_TIMEOUT_SECONDS = 22.0;
    private static final double SETTLE_SECONDS = 0.35;
    private static final double MAX_ERROR_GROWTH_INCHES = 4.0;

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
    private double commandedFrontLeft;
    private double commandedFrontRight;
    private double commandedBackLeft;
    private double commandedBackRight;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftPod = hardwareMap.get(DcMotor.class, "intake_motor");
        rightPod = hardwareMap.get(DcMotor.class, "odo_right");
        centerPod = hardwareMap.get(DcMotor.class, "odo_center");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        preparePod(leftPod);
        preparePod(rightPod);
        preparePod(centerPod);
        setDriveBrake();
        stopDrive();
        resetPose();

        telemetry.addLine("LOW-SPEED CLOSED-LOOP TEST");
        telemetry.addLine("Clear a 4 x 4 ft L-shaped path; stay ready to press STOP");
        telemetry.addLine("Path: forward 48, left 48, then retrace to start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        resetPose();

        try {
            if (!driveToPose(48, 0, 0, "Forward 48")) return;
            if (!driveToPose(48, 48, 0, "Left 48")) return;
            if (!driveToPose(48, 0, 0, "Right 48")) return;
            if (!driveToPose(0, 0, 0, "Backward 48")) return;
            RobotLog.ii(TAG, "COMPLETE pose x=%.2f y=%.2f heading=%.1f",
                    x, y, Math.toDegrees(heading));
        } finally {
            stopDrive();
        }

        while (opModeIsActive()) {
            updatePose();
            showTelemetry("Complete", 0, 0, 0, 0);
            telemetry.addLine("Press STOP after recording the final pose");
            telemetry.update();
            idle();
        }
    }

    private boolean driveToPose(double targetX, double targetY,
                                double targetHeadingDegrees, String label) {
        double targetHeading = Math.toRadians(targetHeadingDegrees);
        ElapsedTime timeout = new ElapsedTime();
        ElapsedTime settled = new ElapsedTime();
        boolean insideTolerance = false;
        double initialPositionError = Math.hypot(targetX - x, targetY - y);

        while (opModeIsActive() && timeout.seconds() < WAYPOINT_TIMEOUT_SECONDS) {
            updatePose();
            double errorX = targetX - x;
            double errorY = targetY - y;
            double errorHeading = normalizeRadians(targetHeading - heading);
            double positionError = Math.hypot(errorX, errorY);

            if (timeout.seconds() > 0.75
                    && positionError > initialPositionError + MAX_ERROR_GROWTH_INCHES) {
                stopDrive();
                RobotLog.ee(TAG,
                        "ABORT step=%s pose=(%.2f,%.2f,%.1f) posError=%.2f headingError=%.1f",
                        label, x, y, Math.toDegrees(heading), positionError,
                        Math.toDegrees(errorHeading));
                showTelemetry("ABORT: moving away", targetX, targetY,
                        targetHeadingDegrees, timeout.seconds());
                telemetry.addData("Remaining position error", "%.2f in", positionError);
                telemetry.addData("Remaining heading error", "%+.1f deg",
                        Math.toDegrees(errorHeading));
                telemetry.addLine("Press STOP after recording these values");
                telemetry.update();
                holdFailureDisplay();
                return false;
            }

            if (positionError <= POSITION_TOLERANCE_INCHES
                    && Math.abs(errorHeading) <= HEADING_TOLERANCE_RADIANS) {
                stopDrive();
                if (!insideTolerance) {
                    settled.reset();
                    insideTolerance = true;
                }
                if (settled.seconds() >= SETTLE_SECONDS) {
                    RobotLog.ii(TAG, "WAYPOINT step=%s pose=(%.2f,%.2f,%.1f) time=%.2f",
                            label, x, y, Math.toDegrees(heading), timeout.seconds());
                    return true;
                }
            } else {
                insideTolerance = false;
                double cos = Math.cos(heading);
                double sin = Math.sin(heading);
                double robotForwardError = errorX * cos + errorY * sin;
                double robotLeftError = -errorX * sin + errorY * cos;

                double physicalForward = Range.clip(robotForwardError * TRANSLATION_KP,
                        -MAX_FORWARD_POWER, MAX_FORWARD_POWER);
                double physicalLeft = Range.clip(robotLeftError * TRANSLATION_KP,
                        -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
                if (positionError <= POSITION_TOLERANCE_INCHES) {
                    physicalForward = 0;
                    physicalLeft = 0;
                } else {
                    if (Math.abs(robotForwardError) <= AXIS_CORRECTION_DEADBAND_INCHES) {
                        physicalForward = 0;
                    } else if (Math.abs(physicalForward) < MIN_FORWARD_POWER) {
                        physicalForward = Math.copySign(MIN_FORWARD_POWER, physicalForward);
                    }
                    if (Math.abs(robotLeftError) <= AXIS_CORRECTION_DEADBAND_INCHES) {
                        physicalLeft = 0;
                    } else if (Math.abs(physicalLeft) < MIN_STRAFE_POWER) {
                        physicalLeft = Math.copySign(MIN_STRAFE_POWER, physicalLeft);
                    }
                }

                double physicalCcw = Range.clip(errorHeading * HEADING_KP,
                        -MAX_TURN_POWER, MAX_TURN_POWER);
                if (Math.abs(errorHeading) <= HEADING_TOLERANCE_RADIANS) {
                    physicalCcw = 0;
                } else if (Math.abs(physicalCcw) < MIN_TURN_POWER) {
                    physicalCcw = Math.copySign(MIN_TURN_POWER, physicalCcw);
                }

                // This robot's mecanum mix uses negative power for physical forward.
                setMecanum(-physicalForward, physicalLeft, physicalCcw);
            }

            showTelemetry(label, targetX, targetY, targetHeadingDegrees, timeout.seconds());
            telemetry.update();
            idle();
        }

        stopDrive();
        updatePose();
        RobotLog.ee(TAG,
                "TIMEOUT step=%s pose=(%.2f,%.2f,%.1f) posError=%.2f headingError=%.1f",
                label, x, y, Math.toDegrees(heading),
                Math.hypot(targetX - x, targetY - y),
                Math.toDegrees(normalizeRadians(targetHeading - heading)));
        showTelemetry("TIMEOUT: " + label, targetX, targetY,
                targetHeadingDegrees, timeout.seconds());
        telemetry.addData("Remaining position error", "%.2f in",
                Math.hypot(targetX - x, targetY - y));
        telemetry.addData("Remaining heading error", "%+.1f deg",
                Math.toDegrees(normalizeRadians(targetHeading - heading)));
        telemetry.addLine("Press STOP after recording these values");
        telemetry.update();
        holdFailureDisplay();
        return false;
    }

    private void holdFailureDisplay() {
        while (opModeIsActive()) {
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

        double deltaHeading = (deltaRight - deltaLeft) / PARALLEL_POD_SPACING_INCHES;
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

    private void preparePod(DcMotor pod) {
        pod.setPower(0);
        pod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private int readLeft() { return -leftPod.getCurrentPosition(); }
    private int readRight() { return rightPod.getCurrentPosition(); }
    private int readCenter() { return -centerPod.getCurrentPosition(); }

    private void setMecanum(double forwardMix, double leftMix, double ccwMix) {
        double fl = forwardMix + leftMix + ccwMix;
        double fr = forwardMix - leftMix - ccwMix;
        double bl = forwardMix - leftMix + ccwMix;
        double br = forwardMix + leftMix - ccwMix;
        double scale = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        commandedFrontLeft = fl / scale;
        commandedFrontRight = fr / scale;
        commandedBackLeft = bl / scale;
        commandedBackRight = br / scale;
        frontLeft.setPower(commandedFrontLeft);
        frontRight.setPower(commandedFrontRight);
        backLeft.setPower(commandedBackLeft);
        backRight.setPower(commandedBackRight);
    }

    private void showTelemetry(String step, double targetX, double targetY,
                               double targetHeading, double seconds) {
        telemetry.addData("Step", step);
        telemetry.addData("Target X / Y", "%.1f / %.1f in", targetX, targetY);
        telemetry.addData("Target heading", "%.1f deg", targetHeading);
        telemetry.addData("Pose X / Y", "%+.2f / %+.2f in", x, y);
        telemetry.addData("Heading", "%+.1f deg", Math.toDegrees(heading));
        telemetry.addData("Power FL / FR", "%+.2f / %+.2f",
                commandedFrontLeft, commandedFrontRight);
        telemetry.addData("Power BL / BR", "%+.2f / %+.2f",
                commandedBackLeft, commandedBackRight);
        telemetry.addData("Step time", "%.1f / %.1f sec", seconds, WAYPOINT_TIMEOUT_SECONDS);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void setDriveBrake() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void stopDrive() { setMecanum(0, 0, 0); }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
