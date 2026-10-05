package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Read-only three-wheel odometry pose validation for Jimothy. */
@TeleOp(name = "BioBuzz Odometry Pose Test", group = "Diagnostics")
public class BioBuzzOdometryPoseTest extends OpMode {
    private static final double LEFT_TICKS_PER_INCH = 509.66;
    private static final double RIGHT_TICKS_PER_INCH = 507.60;
    private static final double CENTER_TICKS_PER_INCH = 507.92;
    private static final double PARALLEL_POD_SPACING_INCHES = 12.263;
    private static final double CENTER_POD_X_INCHES = -6.718;
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private DcMotor leftPod;
    private DcMotor rightPod;
    private DcMotor centerPod;
    private IMU imu;

    private int previousLeft;
    private int previousRight;
    private int previousCenter;
    private double x;
    private double y;
    private double heading;
    private double accumulatedHeading;
    private double previousImuYaw;
    private double accumulatedImuHeading;
    private boolean previousDpadUp;

    @Override
    public void init() {
        // The left pod shares Expansion Hub motor/encoder channel 0 with the intake motor.
        leftPod = hardwareMap.get(DcMotor.class, "intake_motor");
        rightPod = hardwareMap.get(DcMotor.class, "odo_right");
        centerPod = hardwareMap.get(DcMotor.class, "odo_center");
        imu = hardwareMap.get(IMU.class, "imu");

        prepareReadOnly(leftPod);
        prepareReadOnly(rightPod);
        prepareReadOnly(centerPod);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                HUB_LOGO_DIRECTION, HUB_USB_DIRECTION)));
        resetPose();

        telemetry.addLine("READ ONLY: drive motors will not be powered");
        telemetry.addLine("D-pad UP = reset pose");
    }

    @Override
    public void start() {
        resetPose();
    }

    @Override
    public void loop() {
        boolean reset = gamepad1.dpad_up || gamepad2.dpad_up;
        if (reset && !previousDpadUp) {
            resetPose();
        }
        previousDpadUp = reset;

        updatePose();
        telemetry.addData("X forward", "%+.2f in", x);
        telemetry.addData("Y left", "%+.2f in", y);
        telemetry.addData("Heading CCW", "%+.1f deg", Math.toDegrees(heading));
        telemetry.addData("Accumulated heading", "%+.1f deg",
                Math.toDegrees(accumulatedHeading));
        telemetry.addData("IMU heading CCW", "%+.1f deg",
                Math.toDegrees(normalizeRadians(accumulatedImuHeading)));
        telemetry.addData("Accumulated odom / IMU", "%+.1f / %+.1f deg",
                Math.toDegrees(accumulatedHeading), Math.toDegrees(accumulatedImuHeading));
        telemetry.addData("Heading error odom-IMU", "%+.1f deg",
                Math.toDegrees(accumulatedHeading - accumulatedImuHeading));
        if (Math.abs(accumulatedImuHeading) >= Math.toRadians(30)) {
            telemetry.addData("Suggested pod spacing", "%.4f in",
                    PARALLEL_POD_SPACING_INCHES
                            * accumulatedHeading / accumulatedImuHeading);
        } else {
            telemetry.addData("Suggested pod spacing", "Turn at least 30 deg");
        }
        telemetry.addData("Corrected pods L / R / C", "%d / %d / %d",
                readLeft(), readRight(), readCenter());
    }

    @Override
    public void stop() {
        leftPod.setPower(0);
        rightPod.setPower(0);
        centerPod.setPower(0);
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
        double midpointHeading = heading + deltaHeading / 2.0;

        x += forward * Math.cos(midpointHeading) - lateral * Math.sin(midpointHeading);
        y += forward * Math.sin(midpointHeading) + lateral * Math.cos(midpointHeading);
        accumulatedHeading += deltaHeading;
        heading = normalizeRadians(heading + deltaHeading);

        double currentImuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        accumulatedImuHeading += normalizeRadians(currentImuYaw - previousImuYaw);
        previousImuYaw = currentImuYaw;
    }

    private void resetPose() {
        previousLeft = readLeft();
        previousRight = readRight();
        previousCenter = readCenter();
        imu.resetYaw();
        previousImuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        x = 0;
        y = 0;
        heading = 0;
        accumulatedHeading = 0;
        accumulatedImuHeading = 0;
    }

    private void prepareReadOnly(DcMotor device) {
        device.setPower(0);
        device.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private int readLeft() {
        return -leftPod.getCurrentPosition();
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
