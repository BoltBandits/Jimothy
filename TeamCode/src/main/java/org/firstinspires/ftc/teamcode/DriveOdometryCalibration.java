package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Manual drivetrain and odometry diagnostic. Motors run only while a button is held. */
@TeleOp(name = "Drive + Odometry Calibration", group = "Jimothy Tests")
public class DriveOdometryCalibration extends OpMode {
    private static final double TEST_POWER = 0.35;
    private static final double LEFT_TICKS_PER_INCH = 503.47;
    private static final double RIGHT_TICKS_PER_INCH = 504.53;
    private static final double CENTER_TICKS_PER_INCH = 510.30;
    private static final double POD_SPACING_INCHES = 12.10;
    private static final double CENTER_POD_X_INCHES = -6.56;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor leftPod;
    private DcMotor rightPod;
    private DcMotor centerPod;
    private DcMotor intakeMotor;

    private int zeroLeft;
    private int zeroRight;
    private int zeroCenter;
    private boolean lastStart;

    @Override
    public void init() {
        RobotHookups hookups = RobotHookups.JIMOTHY_LEGACY;
        frontLeft = hardwareMap.get(DcMotor.class, RobotHookups.Names.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, RobotHookups.Names.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, RobotHookups.Names.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, RobotHookups.Names.BACK_RIGHT);
        leftPod = hardwareMap.get(DcMotor.class, hookups.leftOdometry);
        rightPod = hardwareMap.get(DcMotor.class, hookups.rightOdometry);
        centerPod = hardwareMap.get(DcMotor.class, hookups.strafeOdometry);
        intakeMotor = hardwareMap.get(DcMotor.class, RobotHookups.Names.SWEEPER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        prepareDriveMotor(frontLeft);
        prepareDriveMotor(frontRight);
        prepareDriveMotor(backLeft);
        prepareDriveMotor(backRight);
        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0.0);
        resetEncoderBaselines();

        telemetry.addLine("Hold one button at a time after Start");
        telemetry.addLine("A=forward  B=right strafe  X=forward-right diagonal");
        telemetry.addLine("Y=CCW rotate  LB=left strafe  RB=CW rotate");
        telemetry.addLine("RT=intake-only interference test (drive locked)");
        telemetry.addLine("START resets encoder deltas");
        telemetry.update();
    }

    @Override
    public void loop() {
        String motion = "STOPPED";
        double forward = 0.0;
        double strafe = 0.0;
        double turn = 0.0;

        boolean intakeTest = gamepad1.right_trigger > 0.20;
        if (intakeTest) {
            motion = "INTAKE ONLY - DRIVE LOCKED";
        } else if (gamepad1.a) {
            motion = "FORWARD";
            forward = TEST_POWER;
        } else if (gamepad1.b) {
            motion = "RIGHT STRAFE";
            strafe = TEST_POWER;
        } else if (gamepad1.x) {
            motion = "FORWARD-RIGHT DIAGONAL";
            forward = TEST_POWER;
            strafe = TEST_POWER;
        } else if (gamepad1.y) {
            motion = "CCW ROTATE";
            turn = -TEST_POWER;
        } else if (gamepad1.left_bumper) {
            motion = "LEFT STRAFE";
            strafe = -TEST_POWER;
        } else if (gamepad1.right_bumper) {
            motion = "CW ROTATE";
            turn = TEST_POWER;
        }
        setMecanum(forward, strafe, turn);
        intakeMotor.setPower(intakeTest ? gamepad1.right_trigger : 0.0);

        if (gamepad1.start && !lastStart) resetEncoderBaselines();
        lastStart = gamepad1.start;

        int leftTicks = readLeft() - zeroLeft;
        int rightTicks = readRight() - zeroRight;
        int centerTicks = readCenter() - zeroCenter;
        double leftInches = leftTicks / LEFT_TICKS_PER_INCH;
        double rightInches = rightTicks / RIGHT_TICKS_PER_INCH;
        double centerInches = centerTicks / CENTER_TICKS_PER_INCH;
        double headingRadians = (leftInches - rightInches) / POD_SPACING_INCHES;
        double forwardInches = (leftInches + rightInches) / 2.0;
        double lateralInches = centerInches - CENTER_POD_X_INCHES * headingRadians;

        telemetry.addData("Motion", motion);
        telemetry.addData("Intake power", "%.2f", intakeMotor.getPower());
        telemetry.addData("Command F / S / T", "%+.2f / %+.2f / %+.2f",
                forward, strafe, turn);
        telemetry.addData("Raw delta L / R / C", "%+d / %+d / %+d",
                leftTicks, rightTicks, centerTicks);
        telemetry.addData("Pod inches L / R / C", "%+.2f / %+.2f / %+.2f",
                leftInches, rightInches, centerInches);
        telemetry.addData("Calculated forward / lateral", "%+.2f / %+.2f in",
                forwardInches, lateralInches);
        telemetry.addData("Calculated heading", "%+.1f deg",
                Math.toDegrees(headingRadians));
        telemetry.addLine("A=fwd B=right X=diag Y=CCW LB=left RB=CW RT=intake START=reset");
        telemetry.update();
    }

    @Override
    public void stop() {
        setMecanum(0.0, 0.0, 0.0);
        intakeMotor.setPower(0.0);
    }

    private void resetEncoderBaselines() {
        zeroLeft = readLeft();
        zeroRight = readRight();
        zeroCenter = readCenter();
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

    private void prepareDriveMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0.0);
    }

    private void setMecanum(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;
        double scale = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / scale);
        frontRight.setPower(fr / scale);
        backLeft.setPower(bl / scale);
        backRight.setPower(br / scale);
    }
}
