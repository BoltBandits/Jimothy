package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/** Field-centric mecanum drive using Jimothee's parallel odometry pods. */
@TeleOp(name = "Jimothee Field-Centric TeleOp", group = "Jimothee")
public class JimotheeFieldCentricTeleOp extends LinearOpMode {
    private static final double NORMAL_SCALE = 1.0;
    private static final double SLOW_SCALE = 0.35;
    private static final double STICK_DEAD_ZONE = 0.06;
    private static final double LEFT_TICKS_PER_INCH = 509.66;
    private static final double RIGHT_TICKS_PER_INCH = 507.60;
    private static final double POD_SPACING_INCHES = 12.263;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor leftPod;
    private DcMotor rightPod;
    private int previousLeftTicks;
    private int previousRightTicks;
    private double heading;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftPod = hardwareMap.get(DcMotor.class, "intake_motor");
        rightPod = hardwareMap.get(DcMotor.class, "odo_right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        prepareMotor(frontLeft);
        prepareMotor(frontRight);
        prepareMotor(backLeft);
        prepareMotor(backRight);
        preparePod(leftPod);
        preparePod(rightPod);
        resetOdometryHeading();

        telemetry.addLine("Jimothee field-centric drive ready");
        telemetry.addLine("Left stick: field translation; right stick X: turn");
        telemetry.addLine("Heading source: left/right odometry pods");
        telemetry.addLine("Options/Start: reset field heading; LB: slow mode");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        resetOdometryHeading();

        boolean previousReset = false;
        try {
            while (opModeIsActive()) {
                boolean resetPressed = gamepad1.options || gamepad1.start;
                if (resetPressed && !previousReset) resetOdometryHeading();
                previousReset = resetPressed;

                updateOdometryHeading();
                double fieldForward = applyDeadZone(-gamepad1.left_stick_y);
                double fieldLeft = applyDeadZone(-gamepad1.left_stick_x);
                double ccw = applyDeadZone(-gamepad1.right_stick_x);

                // Rotate the field-frame command into the robot frame.
                double cos = Math.cos(heading);
                double sin = Math.sin(heading);
                double robotForward = fieldForward * cos + fieldLeft * sin;
                double robotLeft = -fieldForward * sin + fieldLeft * cos;
                double scale = gamepad1.left_bumper ? SLOW_SCALE : NORMAL_SCALE;

                // Jimothee requires negative mecanum mix for physical forward.
                setMecanum(-robotForward * scale, robotLeft * scale, ccw * scale);

                telemetry.addData("Mode", gamepad1.left_bumper ? "Slow" : "Normal");
                telemetry.addData("Field heading", "%+.1f deg", Math.toDegrees(heading));
                telemetry.addData("Odometry L / R ticks", "%d / %d",
                        readLeftTicks(), readRightTicks());
                telemetry.addData("Field forward / left", "%+.2f / %+.2f",
                        fieldForward, fieldLeft);
                telemetry.addData("Robot forward / left", "%+.2f / %+.2f",
                        robotForward, robotLeft);
                telemetry.update();
                idle();
            }
        } finally {
            setMecanum(0, 0, 0);
        }
    }

    private void prepareMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

    private void preparePod(DcMotor pod) {
        pod.setPower(0);
        pod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void resetOdometryHeading() {
        previousLeftTicks = readLeftTicks();
        previousRightTicks = readRightTicks();
        heading = 0;
    }

    private void updateOdometryHeading() {
        int leftTicks = readLeftTicks();
        int rightTicks = readRightTicks();
        double deltaLeft = (leftTicks - previousLeftTicks) / LEFT_TICKS_PER_INCH;
        double deltaRight = (rightTicks - previousRightTicks) / RIGHT_TICKS_PER_INCH;
        previousLeftTicks = leftTicks;
        previousRightTicks = rightTicks;
        heading = normalizeRadians(
                heading + (deltaRight - deltaLeft) / POD_SPACING_INCHES);
    }

    private int readLeftTicks() {
        return -leftPod.getCurrentPosition();
    }

    private int readRightTicks() {
        return rightPod.getCurrentPosition();
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private double applyDeadZone(double input) {
        if (Math.abs(input) <= STICK_DEAD_ZONE) return 0;
        return Math.copySign(
                (Math.abs(input) - STICK_DEAD_ZONE) / (1.0 - STICK_DEAD_ZONE), input);
    }

    private void setMecanum(double forward, double left, double ccw) {
        double fl = forward + left + ccw;
        double fr = forward - left - ccw;
        double bl = forward - left + ccw;
        double br = forward + left - ccw;
        double denominator = Math.max(1.0,
                Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(Range.clip(fl / denominator, -1, 1));
        frontRight.setPower(Range.clip(fr / denominator, -1, 1));
        backLeft.setPower(Range.clip(bl / denominator, -1, 1));
        backRight.setPower(Range.clip(br / denominator, -1, 1));
    }
}
