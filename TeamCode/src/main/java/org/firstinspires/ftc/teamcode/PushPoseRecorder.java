package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "Push Pose Recorder", group = "Jimothy Calibration")
public class PushPoseRecorder extends LinearOpMode {
    private static final String TAG = "PushPoseRecorder";
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
    private int previousLeft;
    private int previousRight;
    private int previousCenter;
    private double x;
    private double y;
    private double heading;
    private int captureNumber;
    private boolean lastA;
    private boolean lastB;
    private double capturedX;
    private double capturedY;
    private double capturedHeadingDegrees;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, RobotHookups.Names.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, RobotHookups.Names.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, RobotHookups.Names.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, RobotHookups.Names.BACK_RIGHT);
        leftPod = frontLeft;
        rightPod = hardwareMap.get(DcMotor.class, "odo_right");
        centerPod = hardwareMap.get(DcMotor.class, "odo_center");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        preparePushableMotor(frontLeft);
        preparePushableMotor(frontRight);
        preparePushableMotor(backLeft);
        preparePushableMotor(backRight);
        preparePod(rightPod);
        preparePod(centerPod);
        resetPose();

        telemetry.addLine("Push Pose Recorder ready");
        telemetry.addLine("All drive motors are FLOAT and unpowered.");
        telemetry.addLine("Gamepad 1: B capture pose | A reset origin");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        resetPose();
        while (opModeIsActive()) {
            updatePose();
            boolean aPressed = gamepad1.a && !lastA;
            boolean bPressed = gamepad1.b && !lastB;
            lastA = gamepad1.a;
            lastB = gamepad1.b;

            if (aPressed) {
                resetPose();
                RobotLog.ii(TAG, "ORIGIN_RESET");
            }
            if (bPressed) {
                captureNumber++;
                capturedX = x;
                capturedY = y;
                capturedHeadingDegrees = Math.toDegrees(heading);
                RobotLog.ii(TAG, "CAPTURE_%d x=%.2f y=%.2f heading=%.1f",
                        captureNumber, capturedX, capturedY, capturedHeadingDegrees);
                gamepad1.rumble(150);
            }

            telemetry.addData("Live X", "%+.2f in", x);
            telemetry.addData("Live Y", "%+.2f in", y);
            telemetry.addData("Live heading", "%+.1f deg", Math.toDegrees(heading));
            telemetry.addData("Raw pods L / R / C", "%d / %d / %d",
                    readLeft(), readRight(), readCenter());
            telemetry.addLine("Push robot by hand | B capture | A reset origin");
            if (captureNumber > 0) {
                telemetry.addData("Last capture #" + captureNumber,
                        "X=%+.2f Y=%+.2f heading=%+.1f",
                        capturedX, capturedY, capturedHeadingDegrees);
            } else {
                telemetry.addData("Last capture", "none");
            }
            telemetry.update();
            sleep(20);
        }
    }

    private void updatePose() {
        int currentLeft = readLeft();
        int currentRight = readRight();
        int currentCenter = readCenter();
        double deltaRight = (currentRight - previousRight) / RIGHT_TICKS_PER_INCH;
        double deltaCenter = (currentCenter - previousCenter) / CENTER_TICKS_PER_INCH;
        double deltaLeft = (currentLeft - previousLeft) / LEFT_TICKS_PER_INCH;
        previousLeft = currentLeft;
        previousRight = currentRight;
        previousCenter = currentCenter;

        double deltaHeading = (deltaLeft - deltaRight) / POD_SPACING_INCHES;
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
        x = 0.0;
        y = 0.0;
        heading = 0.0;
    }

    private void preparePushableMotor(DcMotor motor) {
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void preparePod(DcMotor pod) {
        pod.setPower(0.0);
        pod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
