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

/** Drives forward 12, right 65, forward 35, then turns right 95 degrees. */
@Autonomous(name = "Jimothy Automonous", group = "StarterBot")
public class BioBuzzForward30Auto extends LinearOpMode {
    private static final String TAG = "JimothyAuto";
    private static final double LEFT_TICKS_PER_INCH = 509.66;
    private static final double RIGHT_TICKS_PER_INCH = 507.60;
    private static final double CENTER_TICKS_PER_INCH = 507.92;
    private static final double POD_SPACING_INCHES = 12.263;
    private static final double CENTER_POD_X_INCHES = -6.718;

    private static final double TRANSLATION_KP = 0.045;
    private static final double HEADING_KP = 0.65;
    private static final double MAX_FORWARD_POWER = 0.35;
    private static final double MAX_STRAFE_POWER = 0.55;
    private static final double MIN_FORWARD_POWER = 0.10;
    private static final double MIN_STRAFE_POWER = 0.16;
    private static final double MIN_TURN_POWER = 0.09;
    private static final double MAX_TURN_POWER = 0.28;
    private static final double POSITION_TOLERANCE_INCHES = 1.25;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.0);
    private static final double WAYPOINT_TIMEOUT_SECONDS = 22.0;
    private static final double SHOOTER_VELOCITY = 320.0;
    private static final double SHOOTER_SPINUP_SECONDS = 1.0;
    private static final double MIN_SHOOTER_FEED_VELOCITY = 315.0;
    private static final double FEED_PULSE_SECONDS = 0.35;

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

        telemetry.addLine("Ready: forward 12, right 65, forward 35, right 95");
        telemetry.addLine("Keep the entire path clear");
        addShootingTelemetry();
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        resetPose();

        try {
            if (!driveToPose(12, 0, 0, "Forward 12")) return;
            if (!driveToPose(12, -65, 0, "Strafe right 65")) return;
            if (!driveToPose(47, -65, 0, "Forward 35")) return;
            if (!driveToPose(47, -65, -95, "Turn right 95")) return;
            RobotLog.ii(TAG, "Turn complete; spinning up shooter");
            telemetry.addLine("Turn complete; spinning up shooter");
            addShootingTelemetry();
            telemetry.update();
            startShooter();
            if (!waitForShooterSpinup()) return;
            shootWithRecovery();
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

    private void shootWithRecovery() {
        boolean firstBall = true;

        while (opModeIsActive()) {
            // Preserve the requested one-second initial feed. After that, wait for
            // the wheel to recover to at least 315 ticks/s before feeding again.
            if (!firstBall) {
                shootingState = "WINDMILL WAITING";
                windmill.setPower(0);
                while (opModeIsActive()
                        && shooter.getVelocity() < MIN_SHOOTER_FEED_VELOCITY) {
                    shooter.setVelocity(SHOOTER_VELOCITY);
                    telemetry.addData("Step", "Recover shooter before next ball");
                    telemetry.addData("Shooter velocity", "%.1f ticks/s",
                            shooter.getVelocity());
                    telemetry.addData("Target / feed minimum", "%.0f / %.0f ticks/s",
                            SHOOTER_VELOCITY, MIN_SHOOTER_FEED_VELOCITY);
                    telemetry.addLine("Windmill: WAITING");
                    addShootingTelemetry();
                    telemetry.update();
                    idle();
                }
            }

            if (!opModeIsActive()) break;

            ElapsedTime pulseTimer = new ElapsedTime();
            shootingState = "WINDMILL FEEDING";
            windmill.setPower(-1);
            RobotLog.ii(TAG, "Feeding ball at shooter velocity %.0f",
                    shooter.getVelocity());
            while (opModeIsActive()
                    && pulseTimer.seconds() < FEED_PULSE_SECONDS) {
                shooter.setVelocity(SHOOTER_VELOCITY);
                telemetry.addData("Step", "Feed one ball");
                telemetry.addData("Shooter velocity", "%.1f ticks/s",
                        shooter.getVelocity());
                telemetry.addData("Target / feed minimum", "%.0f / %.0f ticks/s",
                        SHOOTER_VELOCITY, MIN_SHOOTER_FEED_VELOCITY);
                telemetry.addLine("Windmill: FEEDING");
                addShootingTelemetry();
                telemetry.addData("Feed pulse", "%.2f / %.2f sec",
                        pulseTimer.seconds(), FEED_PULSE_SECONDS);
                telemetry.update();
                idle();
            }
            windmill.setPower(0);
            firstBall = false;
        }

        shooter.setVelocity(0);
        windmill.setPower(0);
        RobotLog.ii(TAG, "Pulsed shooting stopped with OpMode");
    }

    private boolean driveToPose(double targetX, double targetY,
                                double targetHeadingDegrees, String step) {
        ElapsedTime timer = new ElapsedTime();
        double targetHeading = Math.toRadians(targetHeadingDegrees);

        while (opModeIsActive() && timer.seconds() < WAYPOINT_TIMEOUT_SECONDS) {
            updatePose();
            double errorX = targetX - x;
            double errorY = targetY - y;
            double positionError = Math.hypot(errorX, errorY);
            double headingError = normalizeRadians(targetHeading - heading);

            if (positionError <= POSITION_TOLERANCE_INCHES
                    && Math.abs(headingError) <= HEADING_TOLERANCE_RADIANS) {
                stopDrive();
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
        telemetry.addData("Windmill feed minimum", "%.0f ticks/s",
                MIN_SHOOTER_FEED_VELOCITY);
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
