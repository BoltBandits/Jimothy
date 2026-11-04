package org.firstinspires.ftc.teamcode;

import android.util.Size;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@Autonomous(name = "Red Auto Op Side", group = "Jimothy")
public class RedAutoOpSide extends LinearOpMode {
    private static final String TAG = "RedAutoOpSide";

    // Fresh hand-pushed route: forward, robot-right strafe, then forward.
    // The first two points are intentional square corners, not look-aheads.
    private static final double PATH_1_X = 105.12;
    private static final double PATH_1_Y = -0.92;
    private static final double PATH_1_HEADING = 0.8;
    private static final double PATH_2_X = 105.94;
    private static final double PATH_2_Y = -34.35;
    private static final double PATH_2_HEADING = 2.0;
    private static final double TARGET_X = 123.09;
    private static final double TARGET_Y = -32.87;
    private static final double TARGET_HEADING_DEGREES = 1.4;
    private static final double POSITION_TOLERANCE = 1.25;
    private static final double HEADING_TOLERANCE = Math.toRadians(2.0);
    private static final double DRIVE_TIMEOUT_SECONDS = 22.0;
    private static final double TRANSLATION_KP = 0.075;
    private static final double HEADING_KP = 0.90;
    private static final double MIN_FORWARD = 0.10;
    private static final double MIN_STRAFE = 0.16;
    private static final double MIN_TURN = 0.19;
    private static final double MAX_TURN = 0.70;

    private static final double LEFT_TICKS_PER_INCH = 509.66;
    private static final double RIGHT_TICKS_PER_INCH = 507.60;
    private static final double CENTER_TICKS_PER_INCH = 507.92;
    private static final double POD_SPACING_INCHES = 12.263;
    private static final double CENTER_POD_X_INCHES = -6.718;

    private static final double SHOOTER_SPEED_FRACTION = 0.49954806;
    private static final double SHOOTER_READY_FRACTION = 0.97;
    private static final double SHOOTER_SPINUP_TIMEOUT_SECONDS = 3.0;
    private static final double SHOOTING_SECONDS = 5.5;
    private static final double SHOOTER_P = 40.0;
    private static final double SHOOTER_F = 12.5;

    private static final double UPRIGHT_ROLL_TOLERANCE_DEGREES = 10.0;
    private static final int REQUIRED_STABLE_FRAMES = 3;
    private static final double AUTO_PERIOD_SECONDS = 30.0;
    private static final double FINAL_MOVE_REMAINING_SECONDS = 6.0;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor leftPod;
    private DcMotor rightPod;
    private DcMotor centerPod;
    private DcMotorEx shooter;
    private CRServo windmill;
    private AprilTagProcessor aprilTags;
    private VisionPortal webcamPortal;
    private final ElapsedTime autonomousTimer = new ElapsedTime();

    private int previousLeft;
    private int previousRight;
    private int previousCenter;
    private double x;
    private double y;
    private double heading;
    private double shooterVelocity;
    private double shooterReadyVelocity;
    private double finalPathOriginX;
    private double finalPathOriginY;
    private double finalPathOriginHeading;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("Auto", "%s AUTO OP SIDE READY", targetColorLabel());
        telemetry.addData("First position", "X %.2f, Y %.2f, H %.1f",
                TARGET_X, TARGET_Y, TARGET_HEADING_DEGREES);
        telemetry.addData("At position", "wait indefinitely for upright %s AprilTag",
                targetColorLabel().toLowerCase());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        autonomousTimer.reset();
        resetPose();

        try {
            if (!driveToPose(PATH_1_X, PATH_1_Y, PATH_1_HEADING,
                    1.75, Math.toRadians(4.0), true,
                    "Recorded path 1/3")) return;
            if (!driveToPose(PATH_2_X, PATH_2_Y, PATH_2_HEADING,
                    1.75, Math.toRadians(4.0), true,
                    "Recorded path 2/3")) return;
            if (!driveToPose(TARGET_X, TARGET_Y, TARGET_HEADING_DEGREES,
                    POSITION_TOLERANCE, HEADING_TOLERANCE, true,
                    "Recorded shooting position")) return;
            if (!opModeIsActive()) return;

            stopDrive();
            shooter.setVelocity(shooterVelocity);
            if (!waitForUprightTargetTag()) return;
            if (!waitForShooterReady()) return;
            shootFourBalls();
            if (!followPostShotPath()) return;
            if (!waitForFinalMoveTime()) return;
            if (!followFinalPath()) return;
            holdAfterShooting();
        } finally {
            stopDrive();
            shooter.setVelocity(0);
            windmill.setPower(0);
            if (webcamPortal != null) webcamPortal.close();
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
        shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        windmill = hardwareMap.get(CRServo.class, "windmill_servo");

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

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F));
        shooterVelocity = shooter.getMotorType().getMaxRPM()
                * shooter.getMotorType().getTicksPerRev() / 60.0
                * SHOOTER_SPEED_FRACTION;
        shooterReadyVelocity = shooterVelocity * SHOOTER_READY_FRACTION;
        shooter.setVelocity(0);
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        windmill.setPower(0);

        aprilTags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .build();
        aprilTags.setDecimation(1.0f);
        webcamPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTags)
                .build();
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
        return driveToPose(targetX, targetY, targetHeadingDegrees,
                positionTolerance, headingTolerance, stopAtTarget, step,
                DRIVE_TIMEOUT_SECONDS);
    }

    private boolean driveToPose(double targetX, double targetY,
                                double targetHeadingDegrees,
                                double positionTolerance,
                                double headingTolerance,
                                boolean stopAtTarget, String step,
                                double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        double targetHeading = Math.toRadians(targetHeadingDegrees);
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            updatePose();
            double errorX = targetX - x;
            double errorY = targetY - y;
            double positionError = Math.hypot(errorX, errorY);
            double headingError = normalize(targetHeading - heading);
            if (positionError <= positionTolerance
                    && Math.abs(headingError) <= headingTolerance) {
                RobotLog.ii(TAG, "WAYPOINT_REACHED step=%s pose=(%.2f,%.2f,%.1f)",
                        step, x, y, Math.toDegrees(heading));
                if (stopAtTarget) stopDrive();
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
            telemetry.addData("Time", "%.1f / %.1f", timer.seconds(), timeoutSeconds);
            telemetry.update();
            idle();
        }
        stopDrive();
        if (!opModeIsActive()) return false;
        RobotLog.ww(TAG, "WAYPOINT_TIMEOUT step=%s pose=(%.2f,%.2f,%.1f); continuing",
                step, x, y, Math.toDegrees(heading));
        return true;
    }

    protected String targetColorLabel() {
        return "RED";
    }

    protected boolean isTargetCluster(String name) {
        return name.equalsIgnoreCase("RED SCORING")
                || name.equalsIgnoreCase("RED AUDIENCE");
    }

    protected boolean isTargetSingleTag(int id) {
        return id >= 30 && id <= 37;
    }

    private boolean waitForUprightTargetTag() {
        int stableFrames = 0;
        while (opModeIsActive()) {
            shooter.setVelocity(shooterVelocity);
            boolean uprightTargetVisible = false;
            String detected = "NONE";
            double roll = Double.NaN;
            List<AprilTagDetection> detections = aprilTags.getDetections();
            for (AprilTagDetection detection : detections) {
                boolean target;
                if (detection instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection cluster =
                            (AprilTagClusterDetection) detection;
                    String name = cluster.metadata == null ? "" : cluster.metadata.name;
                    target = isTargetCluster(name);
                    detected = name;
                } else if (detection instanceof AprilTagSingleDetection) {
                    int id = ((AprilTagSingleDetection) detection).id;
                    target = isTargetSingleTag(id);
                    detected = "tag " + id;
                } else {
                    target = false;
                }
                if (target && detection.ftcPose != null
                        && Math.abs(detection.ftcPose.roll)
                        <= UPRIGHT_ROLL_TOLERANCE_DEGREES) {
                    uprightTargetVisible = true;
                    roll = detection.ftcPose.roll;
                    break;
                }
            }
            stableFrames = uprightTargetVisible ? stableFrames + 1 : 0;
            telemetry.addData("Step", "Waiting for upright %s AprilTag",
                    targetColorLabel().toLowerCase());
            telemetry.addData("Webcam", webcamPortal.getCameraState());
            telemetry.addData("Detected", detected);
            telemetry.addData("Roll", Double.isNaN(roll) ? "--" : "%+.1f deg", roll);
            telemetry.addData("Stable frames", "%d / %d", stableFrames, REQUIRED_STABLE_FRAMES);
            telemetry.addData("Shooter", "%.0f / %.0f ticks/s",
                    Math.abs(shooter.getVelocity()), shooterVelocity);
            telemetry.update();
            if (stableFrames >= REQUIRED_STABLE_FRAMES) {
                RobotLog.ii(TAG, "UPRIGHT_%s_TAG_CONFIRMED %s roll=%.1f",
                        targetColorLabel(), detected, roll);
                return true;
            }
            idle();
        }
        return false;
    }

    private boolean waitForShooterReady() {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SHOOTER_SPINUP_TIMEOUT_SECONDS) {
            shooter.setVelocity(shooterVelocity);
            double actual = Math.abs(shooter.getVelocity());
            if (actual >= shooterReadyVelocity) return true;
            telemetry.addData("Step", "%s tag found - shooter spin-up",
                    targetColorLabel());
            telemetry.addData("Shooter", "%.0f / %.0f ticks/s", actual, shooterVelocity);
            telemetry.update();
            idle();
        }
        return opModeIsActive();
    }

    private void shootFourBalls() {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SHOOTING_SECONDS) {
            shooter.setVelocity(shooterVelocity);
            windmill.setPower(-1.0);
            telemetry.addData("Step", "Shooting four balls");
            telemetry.addData("Shooting time", "%.1f / %.1f sec",
                    timer.seconds(), SHOOTING_SECONDS);
            telemetry.addData("Shooter", "%.0f / %.0f ticks/s",
                    Math.abs(shooter.getVelocity()), shooterVelocity);
            telemetry.update();
            idle();
        }
        windmill.setPower(0);
        shooter.setVelocity(0);
    }

    private boolean followPostShotPath() {
        double originX = x;
        double originY = y;
        double originHeading = heading;
        double cos = Math.cos(originHeading);
        double sin = Math.sin(originHeading);
        finalPathOriginX = originX + (-9.95) * cos - (-21.81) * sin;
        finalPathOriginY = originY + (-9.95) * sin + (-21.81) * cos;
        finalPathOriginHeading = normalize(originHeading + Math.toRadians(0.5));

        double departureTime = AUTO_PERIOD_SECONDS - FINAL_MOVE_REMAINING_SECONDS;
        double availableSeconds = Math.max(0.10,
                departureTime - autonomousTimer.seconds());
        return driveToPose(finalPathOriginX, finalPathOriginY,
                Math.toDegrees(finalPathOriginHeading),
                POSITION_TOLERANCE, Math.toRadians(4.0), true,
                "Fast direct path to second position", availableSeconds);
    }

    private boolean waitForFinalMoveTime() {
        stopDrive();
        double departureTime = AUTO_PERIOD_SECONDS - FINAL_MOVE_REMAINING_SECONDS;
        while (opModeIsActive() && autonomousTimer.seconds() < departureTime) {
            double remaining = Math.max(0.0,
                    AUTO_PERIOD_SECONDS - autonomousTimer.seconds());
            telemetry.addData("Step", "Holding second position");
            telemetry.addData("Auto time", "%.1f / %.1f sec",
                    autonomousTimer.seconds(), AUTO_PERIOD_SECONDS);
            telemetry.addData("Final move begins", "at %.1f sec remaining",
                    FINAL_MOVE_REMAINING_SECONDS);
            telemetry.addData("Time remaining", "%.1f sec", remaining);
            telemetry.update();
            idle();
        }
        return opModeIsActive();
    }

    private boolean followFinalPath() {
        if (!driveToRelativePose(finalPathOriginX, finalPathOriginY,
                finalPathOriginHeading,
                -6.09, 35.69, -2.4,
                5.0, 8.0, false, "Final path look-ahead")) return false;
        return driveToRelativePose(finalPathOriginX, finalPathOriginY,
                finalPathOriginHeading,
                -5.67, 71.88, 1.8,
                POSITION_TOLERANCE, 3.0, true, "Final position");
    }

    private boolean driveToRelativePose(double originX, double originY,
                                        double originHeading,
                                        double relativeX, double relativeY,
                                        double relativeHeadingDegrees,
                                        double positionTolerance,
                                        double headingToleranceDegrees,
                                        boolean stopAtTarget, String step) {
        double cos = Math.cos(originHeading);
        double sin = Math.sin(originHeading);
        double targetX = originX + relativeX * cos - relativeY * sin;
        double targetY = originY + relativeX * sin + relativeY * cos;
        double targetHeadingDegrees = Math.toDegrees(originHeading)
                + relativeHeadingDegrees;
        return driveToPose(targetX, targetY, targetHeadingDegrees,
                positionTolerance, Math.toRadians(headingToleranceDegrees),
                stopAtTarget, step);
    }

    private void holdAfterShooting() {
        stopDrive();
        shooter.setVelocity(0);
        windmill.setPower(0);
        while (opModeIsActive()) {
            telemetry.addData("Step", "AUTO COMPLETE - holding final position");
            telemetry.addData("Pose", "X %+.2f Y %+.2f H %+.1f",
                    x, y, Math.toDegrees(heading));
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
