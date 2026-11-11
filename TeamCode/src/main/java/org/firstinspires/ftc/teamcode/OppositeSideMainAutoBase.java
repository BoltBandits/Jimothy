package org.firstinspires.ftc.teamcode;

import android.util.Size;

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

/** Shared AprilTag-wait and stationary shooting sequence for opposite-side autos. */
abstract class OppositeSideMainAutoBase extends LinearOpMode {
    private static final double SHOOTER_SPEED_FRACTION = 0.49954806;
    private static final double SHOOTER_READY_FRACTION = 0.97;
    private static final double SHOOTER_SPINUP_TIMEOUT_SECONDS = 3.0;
    private static final double SHOOTING_SECONDS = 7.5;
    private static final double SHOOTER_P = 40.0;
    private static final double SHOOTER_F = 12.5;
    private static final double LEFT_TICKS_PER_INCH = 509.66;
    private static final double RIGHT_TICKS_PER_INCH = 507.60;
    private static final double CENTER_TICKS_PER_INCH = 507.92;
    private static final double POD_SPACING_INCHES = 12.263;
    private static final double CENTER_POD_X_INCHES = -6.718;

    private DcMotorEx shooter;
    private CRServo windmill;
    private AprilTagProcessor aprilTags;
    private VisionPortal webcamPortal;
    private double shooterVelocity;
    private double shooterReadyVelocity;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor rightPod;
    private DcMotor centerPod;
    private int previousLeft;
    private int previousRight;
    private int previousCenter;
    private double x;
    private double y;
    private double heading;

    protected abstract String targetTagName();

    protected abstract boolean isTargetSingleTag(int id);

    protected boolean captureNextPositionAfterShooting() {
        return false;
    }

    protected boolean driveToRecordedNextPositionAfterShooting() {
        return false;
    }

    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.addData("Auto", "%s READY", getClass().getSimpleName());
        telemetry.addData("Sequence", "Wait for %s, then shoot four", targetTagName());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        try {
            waitForTargetTag();
            if (!opModeIsActive()) return;
            shooter.setVelocity(shooterVelocity);
            if (!waitForShooterReady()) return;
            shootFourBalls();
            if (captureNextPositionAfterShooting()) {
                recordNextPositionMode();
                return;
            }
            if (driveToRecordedNextPositionAfterShooting()) {
                driveFastToPose(-8.32, 7.00, 0.0);
            }
            while (opModeIsActive()) {
                stopMechanisms();
                telemetry.addData("Step", "%s FOUND - SHOOTING COMPLETE", targetTagName());
                telemetry.update();
                idle();
            }
        } finally {
            stopMechanisms();
            if (webcamPortal != null) webcamPortal.close();
        }
    }

    private void initializeHardware() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        windmill = hardwareMap.get(CRServo.class, "windmill_servo");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, 0.0, 0.0, SHOOTER_F));
        double maxVelocity = shooter.getMotorType().getMaxRPM()
                * shooter.getMotorType().getTicksPerRev() / 60.0;
        shooterVelocity = SHOOTER_SPEED_FRACTION * maxVelocity;
        shooterReadyVelocity = SHOOTER_READY_FRACTION * shooterVelocity;
        shooter.setVelocity(0.0);

        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        windmill.setPower(0.0);

        aprilTags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .build();
        aprilTags.setDecimation(1.0f);
        webcamPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTags)
                .build();

        if (captureNextPositionAfterShooting()
                || driveToRecordedNextPositionAfterShooting()) {
            initializePositionCaptureHardware();
        }
    }

    private boolean waitForShooterReady() {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SHOOTER_SPINUP_TIMEOUT_SECONDS) {
            shooter.setVelocity(shooterVelocity);
            double actual = Math.abs(shooter.getVelocity());
            telemetry.addData("Step", "Shooter spin-up");
            telemetry.addData("Shooter", "%.0f / %.0f ticks/s", actual, shooterVelocity);
            telemetry.update();
            if (actual >= shooterReadyVelocity) return true;
            idle();
        }
        return false;
    }

    private void shootFourBalls() {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SHOOTING_SECONDS) {
            shooter.setVelocity(shooterVelocity);
            double actual = Math.abs(shooter.getVelocity());
            boolean ready = actual >= shooterReadyVelocity;
            windmill.setPower(ready ? -1.0 : 0.0);
            telemetry.addData("Step", "Shooting four balls");
            telemetry.addData("Shooting time", "%.1f / %.1f sec",
                    timer.seconds(), SHOOTING_SECONDS);
            telemetry.addData("Shooter", "%.0f / %.0f ticks/s", actual, shooterVelocity);
            telemetry.addData("Windmill", ready ? "FEEDING" : "WAITING FOR SPEED");
            telemetry.update();
            idle();
        }
        stopMechanisms();
    }

    private void waitForTargetTag() {
        while (opModeIsActive()) {
            String detected = "NONE";
            for (AprilTagDetection detection : aprilTags.getDetections()) {
                boolean target = false;
                if (detection instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                    detected = cluster.metadata == null ? "unknown cluster"
                            : cluster.metadata.name;
                    target = detected.equalsIgnoreCase(targetTagName());
                } else if (detection instanceof AprilTagSingleDetection) {
                    int id = ((AprilTagSingleDetection) detection).id;
                    detected = "tag " + id;
                    target = isTargetSingleTag(id);
                }
                if (target) return;
            }
            telemetry.addData("Step", "Waiting for %s", targetTagName());
            telemetry.addData("Detected", detected);
            telemetry.addData("Webcam", webcamPortal.getCameraState());
            telemetry.update();
            idle();
        }
    }

    private void initializePositionCaptureHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");
        rightPod = hardwareMap.get(DcMotor.class, "odo_right");
        centerPod = hardwareMap.get(DcMotor.class, "odo_center");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0.0);
        }
        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        previousLeft = frontLeft.getCurrentPosition();
        previousRight = rightPod.getCurrentPosition();
        previousCenter = centerPod.getCurrentPosition();
    }

    private void recordNextPositionMode() {
        stopMechanisms();
        while (opModeIsActive() && !gamepad1.b) {
            updatePose();
            setMecanum(-gamepad1.left_stick_y * 0.40,
                    -gamepad1.left_stick_x * 0.40,
                    -gamepad1.right_stick_x * 0.35);
            telemetry.addData("Mode", "RED OPPOSITE-SIDE NEXT POSITION");
            telemetry.addData("X forward", "%+.2f in", x);
            telemetry.addData("Y left", "%+.2f in", y);
            telemetry.addData("Heading CCW", "%+.1f deg", Math.toDegrees(heading));
            telemetry.addLine("Gamepad 1 sticks move robot; press B to capture");
            telemetry.update();
            idle();
        }
        setMecanum(0, 0, 0);
        updatePose();
        RobotLog.ii("OppositeSideAuto", "NEXT_POSITION x=%.2f y=%.2f heading=%.1f",
                x, y, Math.toDegrees(heading));
        while (opModeIsActive()) {
            telemetry.addData("Mode", "POSE CAPTURED");
            telemetry.addData("RECORDED X forward", "%+.2f in", x);
            telemetry.addData("RECORDED Y left", "%+.2f in", y);
            telemetry.addData("RECORDED heading CCW", "%+.1f deg",
                    Math.toDegrees(heading));
            telemetry.addLine("Write down these values, then press STOP");
            telemetry.update();
            idle();
        }
    }

    private void driveFastToPose(double targetX, double targetY,
                                 double targetHeadingDegrees) {
        ElapsedTime timer = new ElapsedTime();
        double targetHeading = Math.toRadians(targetHeadingDegrees);
        while (opModeIsActive() && timer.seconds() < 10.0) {
            updatePose();
            double errorX = targetX - x;
            double errorY = targetY - y;
            double positionError = Math.hypot(errorX, errorY);
            double headingError = normalize(targetHeading - heading);
            if (positionError <= 1.25 && Math.abs(headingError) <= Math.toRadians(3.0)) {
                break;
            }

            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double forwardError = errorX * cos + errorY * sin;
            double leftError = -errorX * sin + errorY * cos;
            double forward = Range.clip(forwardError * 0.10, -1.0, 1.0);
            double left = Range.clip(leftError * 0.10, -1.0, 1.0);
            double turn = Range.clip(headingError * 1.2, -0.70, 0.70);
            if (Math.abs(forwardError) < 0.75) forward = 0.0;
            if (Math.abs(leftError) < 0.75) left = 0.0;
            if (Math.abs(headingError) < Math.toRadians(2.0)) turn = 0.0;
            setMecanum(forward, -left, -turn);

            telemetry.addData("Step", "Fast drive to recorded final position");
            telemetry.addData("Target", "X %+.2f Y %+.2f H %+.1f",
                    targetX, targetY, targetHeadingDegrees);
            telemetry.addData("Pose", "X %+.2f Y %+.2f H %+.1f",
                    x, y, Math.toDegrees(heading));
            telemetry.addData("Position error", "%.2f in", positionError);
            telemetry.update();
            idle();
        }
        setMecanum(0, 0, 0);
        updatePose();
    }

    private void updatePose() {
        int currentLeft = frontLeft.getCurrentPosition();
        int currentRight = rightPod.getCurrentPosition();
        int currentCenter = centerPod.getCurrentPosition();
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
        heading += deltaHeading;
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

    private double normalize(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private void stopMechanisms() {
        if (shooter != null) shooter.setVelocity(0.0);
        if (windmill != null) windmill.setPower(0.0);
        if (frontLeft != null) setMecanum(0, 0, 0);
    }
}
