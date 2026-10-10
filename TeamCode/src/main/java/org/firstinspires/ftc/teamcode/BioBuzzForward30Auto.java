package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

/** Drives one continuous curved path to x=47, y=-67, heading=-95 degrees. */
@Autonomous(name = "Jimothy Automonous", group = "StarterBot")
public class BioBuzzForward30Auto extends LinearOpMode {
    private static final String TAG = "JimothyAuto";
    private static final double LEFT_TICKS_PER_INCH = 509.66;
    private static final double RIGHT_TICKS_PER_INCH = 507.60;
    private static final double CENTER_TICKS_PER_INCH = 507.92;
    private static final double POD_SPACING_INCHES = 12.263;
    private static final double CENTER_POD_X_INCHES = -6.718;

    private static final double TRANSLATION_KP = 0.060;
    private static final double HEADING_KP = 0.80;
    private static final double MAX_FORWARD_POWER = 0.80;
    private static final double MAX_STRAFE_POWER = 1.00;
    private static final double MIN_FORWARD_POWER = 0.10;
    private static final double MIN_STRAFE_POWER = 0.16;
    private static final double MIN_TURN_POWER = 0.09;
    private static final double MAX_TURN_POWER = 0.60;
    private static final double POSITION_TOLERANCE_INCHES = 1.25;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.0);
    private static final double WAYPOINT_TIMEOUT_SECONDS = 22.0;
    private static final double SHOOTER_VELOCITY = 300.0;
    private static final double SHOOTER_SPINUP_SECONDS = 1.0;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor leftPod;
    private DcMotor rightPod;
    private DcMotor centerPod;
    private DcMotorEx shooter;
    private CRServo windmill;

    private int previousLeft;
    private int previousRight;
    private int previousCenter;
    private double x;
    private double y;
    private double heading;
    private String shootingState = "IDLE";

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftPod = hardwareMap.get(DcMotor.class, "intake_motor");
        rightPod = hardwareMap.get(DcMotor.class, "odo_right");
        centerPod = hardwareMap.get(DcMotor.class, "odo_center");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        windmill = hardwareMap.get(CRServo.class, "windmill_servo");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        prepareDriveMotor(frontLeft);
        prepareDriveMotor(frontRight);
        prepareDriveMotor(backLeft);
        prepareDriveMotor(backRight);
        preparePod(leftPod);
        preparePod(rightPod);
        preparePod(centerPod);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));
        shooter.setVelocity(0);
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        windmill.setPower(0);
        stopDrive();
        resetPose();

        telemetry.addLine("Ready: fast path to X=47, Y=-67, heading=-95");
        telemetry.addLine("Keep the entire path clear");
        addShootingTelemetry();
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        resetPose();

        try {
            // Bias the curve right early to clear the nearby obstacle. This is a
            // pass-through waypoint, so the drivetrain does not stop here.
            if (!driveThroughPose(12, -30, -30, "Curve right early")) return;
            if (!driveToPose(47, -67, -95, "Fast drive + turn")) return;
            RobotLog.ii(TAG, "Final pose reached; spinning up shooter");
            telemetry.addLine("Final pose reached; spinning up shooter");
            addShootingTelemetry();
            telemetry.update();
            startShooter();
            if (!waitForShooterSpinup()) return;
            runShooterAndWindmill();
            if (opModeIsActive()) recordPositionMode();
        } finally {
            stopDrive();
            shooter.setVelocity(0);
            windmill.setPower(0);
            shootingState = "STOPPED";
        }

        telemetry.addLine("Complete");
        telemetry.addData("Final X / Y", "%.1f / %.1f in", x, y);
        addShootingTelemetry();
        telemetry.update();
        sleep(1000);
    }

    private void startShooter() {
        shootingState = "SHOOTER SPIN-UP";
        shooter.setVelocity(SHOOTER_VELOCITY);
        RobotLog.ii(TAG, "Shooter spin-up started");
    }

    private boolean waitForShooterSpinup() {
        ElapsedTime spinupTimer = new ElapsedTime();
        while (opModeIsActive() && spinupTimer.seconds() < SHOOTER_SPINUP_SECONDS) {
            shooter.setVelocity(SHOOTER_VELOCITY);
            double velocity = shooter.getVelocity();
            telemetry.addData("Step", "One-second shooter spin-up");
            telemetry.addData("Shooter", "%.0f / %.0f ticks/s",
                    velocity, SHOOTER_VELOCITY);
            telemetry.addData("Spin-up time", "%.1f / %.1f sec",
                    spinupTimer.seconds(), SHOOTER_SPINUP_SECONDS);
            addShootingTelemetry();
            telemetry.update();
            idle();
        }
        if (!opModeIsActive()) return false;
        RobotLog.ii(TAG, "One-second spin-up complete at %.0f ticks/s",
                shooter.getVelocity());
        return true;
    }

    private void runShooterAndWindmill() {
        shootingState = "SHOOTER + WINDMILL RUNNING";
        windmill.setPower(-1);
        RobotLog.ii(TAG, "Windmill running continuously with shooter");
        while (opModeIsActive() && !gamepad1.a) {
            shooter.setVelocity(SHOOTER_VELOCITY);
            telemetry.addData("Step", "Continuous shooting");
            telemetry.addLine("Press gamepad 1 A to stop shooting and record a new pose");
            addShootingTelemetry();
            telemetry.update();
            idle();
        }

        shooter.setVelocity(0);
        windmill.setPower(0);
        RobotLog.ii(TAG, "Continuous shooting stopped");
    }

    private void recordPositionMode() {
        shootingState = "OFF - RECORDING POSE";
        shooter.setVelocity(0);
        windmill.setPower(0);
        stopDrive();

        // Wait for the A press used to enter this mode to be released.
        while (opModeIsActive() && gamepad1.a) idle();

        while (opModeIsActive() && !gamepad1.b) {
            updatePose();

            // Reduced manual power makes final positioning easier and safer.
            setMecanum(
                    gamepad1.left_stick_y * 0.40,
                    -gamepad1.left_stick_x * 0.40,
                    -gamepad1.right_stick_x * 0.35);

            telemetry.addData("Mode", "POSITION RECORDER");
            telemetry.addData("X forward", "%+.2f in", x);
            telemetry.addData("Y left", "%+.2f in", y);
            telemetry.addData("Heading CCW", "%+.1f deg", Math.toDegrees(heading));
            telemetry.addLine("Drive with gamepad 1; press B to capture this pose");
            addShootingTelemetry();
            telemetry.update();
            idle();
        }

        stopDrive();
        updatePose();
        RobotLog.ii(TAG, "RECORDED POSE x=%.2f y=%.2f heading=%.1f",
                x, y, Math.toDegrees(heading));

        while (opModeIsActive()) {
            telemetry.addData("Mode", "POSE CAPTURED");
            telemetry.addData("RECORDED X forward", "%+.2f in", x);
            telemetry.addData("RECORDED Y left", "%+.2f in", y);
            telemetry.addData("RECORDED heading CCW", "%+.1f deg",
                    Math.toDegrees(heading));
            telemetry.addLine("Write down these values, then press STOP");
            addShootingTelemetry();
            telemetry.update();
            idle();
        }
    }

    private boolean driveThroughPose(double targetX, double targetY,
                                     double targetHeadingDegrees, String step) {
        return driveToPose(targetX, targetY, targetHeadingDegrees, step,
                4.0, Math.toRadians(10.0), false);
    }

    private boolean driveToPose(double targetX, double targetY,
                                double targetHeadingDegrees, String step) {
        return driveToPose(targetX, targetY, targetHeadingDegrees, step,
                POSITION_TOLERANCE_INCHES, HEADING_TOLERANCE_RADIANS, true);
    }

    private boolean driveToPose(double targetX, double targetY,
                                double targetHeadingDegrees, String step,
                                double positionTolerance, double headingTolerance,
                                boolean stopAtTarget) {
        ElapsedTime timer = new ElapsedTime();
        double targetHeading = Math.toRadians(targetHeadingDegrees);

        while (opModeIsActive() && timer.seconds() < WAYPOINT_TIMEOUT_SECONDS) {
            updatePose();
            double errorX = targetX - x;
            double errorY = targetY - y;
            double positionError = Math.hypot(errorX, errorY);
            double headingError = normalizeRadians(targetHeading - heading);

            if (positionError <= positionTolerance
                    && Math.abs(headingError) <= headingTolerance) {
                if (stopAtTarget) stopDrive();
                return true;
            }

            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double robotForwardError = errorX * cos + errorY * sin;
            double robotLeftError = -errorX * sin + errorY * cos;

            double forward = minimumPower(
                    Range.clip(robotForwardError * TRANSLATION_KP,
                            -MAX_FORWARD_POWER, MAX_FORWARD_POWER),
                    MIN_FORWARD_POWER, robotForwardError);
            double left = minimumPower(
                    Range.clip(robotLeftError * TRANSLATION_KP,
                            -MAX_STRAFE_POWER, MAX_STRAFE_POWER),
                    MIN_STRAFE_POWER, robotLeftError);
            double turn = Range.clip(headingError * HEADING_KP,
                    -MAX_TURN_POWER, MAX_TURN_POWER);

            if (Math.abs(robotForwardError) < 0.75) forward = 0;
            if (Math.abs(robotLeftError) < 0.75) left = 0;
            if (Math.abs(headingError) <= HEADING_TOLERANCE_RADIANS) {
                turn = 0;
            } else if (Math.abs(turn) < MIN_TURN_POWER) {
                turn = Math.copySign(MIN_TURN_POWER, turn);
            }

            // This drivetrain requires negative motor mix for physical forward.
            setMecanum(-forward, left, turn);

            telemetry.addData("Step", step);
            telemetry.addData("Target X / Y", "%.1f / %.1f in", targetX, targetY);
            telemetry.addData("Target heading", "%.0f deg", targetHeadingDegrees);
            telemetry.addData("Pose X / Y", "%+.1f / %+.1f in", x, y);
            telemetry.addData("Heading", "%+.1f deg", Math.toDegrees(heading));
            addShootingTelemetry();
            telemetry.update();
            idle();
        }

        stopDrive();
        telemetry.addData("Status", "Timed out during %s", step);
        addShootingTelemetry();
        telemetry.update();
        return false;
    }

    private void addShootingTelemetry() {
        telemetry.addData("Shooting state", shootingState);
        telemetry.addData("Actual shooter speed", "%.1f ticks/s", shooter.getVelocity());
        telemetry.addData("Shooter target", "%.0f ticks/s", SHOOTER_VELOCITY);
    }

    private double minimumPower(double power, double minimum, double error) {
        if (Math.abs(power) < minimum && Math.abs(error) > 0) {
            return Math.copySign(minimum, error);
        }
        return power;
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

    private void prepareDriveMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

    private void preparePod(DcMotor pod) {
        pod.setPower(0);
        pod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private int readLeft() { return -leftPod.getCurrentPosition(); }
    private int readRight() { return rightPod.getCurrentPosition(); }
    private int readCenter() { return -centerPod.getCurrentPosition(); }

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

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
