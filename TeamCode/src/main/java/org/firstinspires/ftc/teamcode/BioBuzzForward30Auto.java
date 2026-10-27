package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Drives one continuous curved path to x=47, y=-67, heading=-95 degrees. */
@Autonomous(name = "Jimothy Automonous", group = "StarterBot")
public class BioBuzzForward30Auto extends LinearOpMode {
    private static final String TAG = "JimothyAuto";
    private static final int LIMELIGHT_PIPELINE = 0;
    private static final long LIMELIGHT_MAX_STALENESS_MS = 250;
    // Match the detector pipeline's 20% threshold so the approach log retains
    // the lower-confidence pollen that completes the vertical flower column.
    private static final double LIMELIGHT_MIN_CONFIDENCE = 0.20;
    private static final double LIMELIGHT_MIN_AREA_PERCENT = 0.03;
    private static final double LIMELIGHT_DISTANCE_SCALE = 30.0;
    private static final double LIMELIGHT_CAMERA_RIGHT_INCHES = 6.5;
    // Updated from measured wall-approach runs: +1.5 degrees stops the lateral
    // correction before the robot overshoots to the flower's right side.
    private static final double POLLEN_COLUMN_TARGET_TX_DEGREES = 1.5;
    private static final double POLLEN_COLUMN_MAX_TX_SPREAD_DEGREES = 3.0;
    private static final double POLLEN_COLUMN_MIN_TY_SPREAD_DEGREES = 6.0;
    private static final double POLLEN_COLUMN_MAX_TY_SPREAD_DEGREES = 14.0;
    private static final double POLLEN_COLUMN_MIN_MEMBER_AREA_PERCENT = 0.05;
    private static final double POLLEN_COLUMN_MIN_GUIDANCE_TX_DEGREES = -5.0;
    private static final double POLLEN_COLUMN_MAX_GUIDANCE_TX_DEGREES = 8.0;
    private static final double POLLEN_COLUMN_STRAFE_KP = 0.020;
    private static final double POLLEN_COLUMN_MIN_STRAFE = 0.16;
    private static final double POLLEN_COLUMN_MAX_STRAFE = 0.18;
    private static final long POLLEN_COLUMN_HOLD_MS = 350;
    private static final double POLLEN_ALIGNMENT_TOLERANCE_DEGREES = 0.7;
    private static final int POLLEN_ALIGNMENT_STABLE_FRAMES = 1;
    private static final double POLLEN_ALIGNMENT_TIMEOUT_SECONDS = 4.0;
    // During the second approach, translation previously outran the roughly
    // 170-degree heading change and the flower entered view too late.
    private static final double SECOND_APPROACH_LARGE_HEADING_ERROR_RADIANS =
            Math.toRadians(35.0);
    private static final double SECOND_APPROACH_TRANSLATION_SCALE_WHILE_TURNING = 0.55;
    private static final double SECOND_APPROACH_TURN_KP = 1.35;
    private static final double SECOND_APPROACH_MAX_TURN_POWER = 0.90;
    private static final double LEFT_TICKS_PER_INCH = 509.66;
    private static final double RIGHT_TICKS_PER_INCH = 507.60;
    private static final double CENTER_TICKS_PER_INCH = 507.92;
    private static final double POD_SPACING_INCHES = 12.263;
    private static final double CENTER_POD_X_INCHES = -6.718;

    private static final double TRANSLATION_KP = 0.075;
    private static final double HEADING_KP = 0.90;
    private static final double MAX_FORWARD_POWER = 1.00;
    private static final double MAX_STRAFE_POWER = 1.00;
    private static final double MIN_FORWARD_POWER = 0.10;
    private static final double MIN_STRAFE_POWER = 0.16;
    // Enough authority to overcome drivetrain static friction during final
    // heading corrections without changing any pose target or tolerance.
    private static final double MIN_TURN_POWER = 0.19;
    private static final double MAX_TURN_POWER = 0.70;
    private static final double POSITION_TOLERANCE_INCHES = 1.25;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.0);
    private static final double WAYPOINT_TIMEOUT_SECONDS = 22.0;
    // BIOBUZZ StarterBot launcher speed: 1250 ticks/s is about 2678 RPM
    // with the launcher's 28-tick motor encoder.
    private static final double SHOOTER_VELOCITY = 1250.0;
    private static final double SHOOTER_READY_VELOCITY = 1200.0;
    private static final double SHOOTER_SPINUP_TIMEOUT_SECONDS = 3.0;
    private static final double SHOOTING_SECONDS = 3.5;
    private static final double AFTER_SHOOTING_HEADING_DEGREES = -86.4;
    private static final double BACK_AFTER_SHOOTING_INCHES = 6.0;
    private static final double STRAFE_AFTER_SHOOTING_RIGHT_INCHES = 41.0;
    private static final double LONG_STRAFE_TIMEOUT_SECONDS = 8.0;
    private static final double INTAKE_FORWARD_DISTANCE_INCHES = 6.7;
    private static final double INTAKE_FORWARD_TOLERANCE_INCHES = 0.25;
    private static final double INTAKE_FORWARD_POWER = 0.60;
    private static final double INTAKE_FORWARD_TIMEOUT_SECONDS = 2.0;
    private static final double INTAKE_AFTER_DRIVE_SECONDS = 0.25;
    // Shift the return curve left from the robot's intake-facing perspective
    // to provide more clearance around the flower.
    private static final double FINAL_X_INCHES = 25.23;
    private static final double FINAL_Y_INCHES = 13.53;
    // The pre-wall run centered the robot on the flower at about X=42, leaving
    // the desired left intake wheel roughly 7 inches too far right. Shift the
    // pre-wall and final targets robot-left (approximately field-X negative).
    private static final double SECOND_FINAL_X_INCHES = 31.95;
    private static final double SECOND_FINAL_Y_INCHES = 51.95;
    private static final double SECOND_PREWALL_Y_INCHES = 40.0;
    private static final double SECOND_FINAL_APPROACH_TIMEOUT_SECONDS = 5.0;
    private static final double SECOND_SHOOTING_HEADING_DEGREES = 94.0;
    private static final double SECOND_POSITION_INTAKE_LEAD_SECONDS = 1.0;
    private static final double SECOND_POSITION_CONTACT_FORWARD_POWER = 0.16;
    private static final double SECOND_POSITION_HOLD_FORWARD_POWER = 0.06;
    private static final double SECOND_POSITION_LATERAL_HOLD_KP = 0.04;
    private static final double SECOND_POSITION_MAX_LATERAL_HOLD_POWER = 0.08;
    private static final double SECOND_POSITION_HEADING_HOLD_KP = 1.50;
    private static final double SECOND_POSITION_MAX_HEADING_HOLD_POWER = 0.35;
    private static final double SECOND_SHOOTING_SECONDS = 4.0;
    // Field pose captured with JimothyParkPoseCapture. Park straight even
    // though the robot was at 30 degrees when the position was measured.
    private static final double PARK_X_INCHES = 0.54;
    private static final double PARK_Y_INCHES = 27.90;
    private static final double PARK_HEADING_DEGREES = 0.0;
    private static final double PARK_DEADLINE_SECONDS = 25.0;
    private static final double EMERGENCY_PARK_TIMEOUT_SECONDS = 4.7;
    private static final double SHOOTER_P = 40.0;
    private static final double SHOOTER_F = 12.5;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intakeMotor;
    private DcMotor leftPod;
    private DcMotor rightPod;
    private DcMotor centerPod;
    private DcMotorEx shooter;
    private CRServo windmill;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private Limelight3A limelight;
    private final ElapsedTime limelightLogTimer = new ElapsedTime();
    private final ElapsedTime pressLogTimer = new ElapsedTime();
    private final ElapsedTime autonomousTimer = new ElapsedTime();
    private boolean parking;
    private boolean logSecondApproachPollen;
    private double latestPollenColumnTx = Double.NaN;
    private long latestPollenColumnNanos;

    private int previousLeft;
    private int previousRight;
    private int previousCenter;
    private double x;
    private double y;
    private double heading;
    private String shootingState = "IDLE";
    private double shooterTargetVelocity = SHOOTER_VELOCITY;
    private String runId = "not-started";

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        // The left odometry encoder is connected to Control Hub encoder port 0.
        // That encoder port shares the front-left DcMotor mapping, while the
        // motor's separate power output continues to drive the front-left wheel.
        leftPod = frontLeft;
        rightPod = hardwareMap.get(DcMotor.class, "odo_right");
        centerPod = hardwareMap.get(DcMotor.class, "odo_center");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        windmill = hardwareMap.get(CRServo.class, "windmill_servo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        prepareDriveMotor(frontLeft);
        prepareDriveMotor(frontRight);
        prepareDriveMotor(backLeft);
        prepareDriveMotor(backRight);
        preparePod(intakeMotor);
        preparePod(rightPod);
        preparePod(centerPod);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F));
        shooter.setVelocity(0);
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        windmill.setPower(0);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();
        stopDrive();
        resetPose();

        telemetry.addLine("Ready: fast path to X=47, Y=-67, heading=-95");
        telemetry.addLine("Keep the entire path clear");
        addShootingTelemetry();
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        runId = Long.toString(System.currentTimeMillis());
        resetPose();
        autonomousTimer.reset();
        parking = false;
        RobotLog.ii(TAG, "RUN_START id=%s target2=(%.2f,%.2f,%.1f)",
                runId, SECOND_FINAL_X_INCHES, SECOND_FINAL_Y_INCHES,
                SECOND_SHOOTING_HEADING_DEGREES);
        // Spin from the start and keep the velocity command active throughout
        // autonomous so the windmill never begins feeding a cold shooter.
        startShooter(SHOOTER_VELOCITY);
        // Run the intake for the entire autonomous. Loose pollen can otherwise
        // wedge beneath the collector before the programmed collection step.
        startIntake();
        RobotLog.ii(TAG, "INTAKE_START id=%s continuous_from_start=true", runId);

        try {
            // Bias the curve right early to clear the nearby obstacle. This is a
            // pass-through waypoint, so the drivetrain does not stop here.
            if (!driveThroughPose(12, -30, -30, "Curve right early")) return;
            if (!driveToPose(47, -67, -95, "Fast drive + turn",
                    2.50, Math.toRadians(4.5), true)) return;
            RobotLog.ii(TAG, "Final pose reached; verifying shooter speed");
            telemetry.addLine("Final pose reached; verifying shooter speed");
            addShootingTelemetry();
            telemetry.update();
            if (!waitForShooterSpinup()) return;
            runShooterAndWindmill();
            if (opModeIsActive()) {
                shootingState = "SHOOTER RUNNING - DRIVING AFTER SHOOTING";
                double backHeadingDegrees = Math.toDegrees(heading);
                double backX = x - BACK_AFTER_SHOOTING_INCHES * Math.cos(heading);
                double backY = y - BACK_AFTER_SHOOTING_INCHES * Math.sin(heading);
                if (!driveToPose(backX, backY, backHeadingDegrees,
                        "Back 6 inches after shooting")) return;

                // Set the heading before measuring the long strafe. Combining
                // this turn with the strafe produced a different arc and amount
                // of wheel slip from run to run.
                if (!driveToPose(x,
                        y,
                        AFTER_SHOOTING_HEADING_DEGREES,
                        "Turn to 41-inch strafe heading")) return;

                double finalHeading = Math.toRadians(AFTER_SHOOTING_HEADING_DEGREES);
                double afterStrafeX = x
                        + STRAFE_AFTER_SHOOTING_RIGHT_INCHES * Math.sin(finalHeading);
                double afterStrafeY = y
                        - STRAFE_AFTER_SHOOTING_RIGHT_INCHES * Math.cos(finalHeading);
                if (!driveToPose(afterStrafeX,
                        afterStrafeY,
                        AFTER_SHOOTING_HEADING_DEGREES,
                        "Strafe right 41 inches after shooting",
                        0.60, Math.toRadians(1.5), true,
                        LONG_STRAFE_TIMEOUT_SECONDS)) return;

                if (!driveForwardWithIntake(INTAKE_FORWARD_DISTANCE_INCHES)) return;
                logSecondApproachPollen = true;
                latestPollenColumnTx = Double.NaN;
                latestPollenColumnNanos = 0;
                limelightLogTimer.reset();
                boolean reachedApproachCurve = driveToPose(FINAL_X_INCHES,
                        FINAL_Y_INCHES,
                        SECOND_SHOOTING_HEADING_DEGREES,
                        "Turn early toward second shooting position",
                        12.0, Math.toRadians(25.0), false);
                if (!reachedApproachCurve) {
                    logSecondApproachPollen = false;
                    return;
                }
                boolean reachedPrewallAlignment = driveToPose(
                        SECOND_FINAL_X_INCHES,
                        SECOND_PREWALL_Y_INCHES,
                        SECOND_SHOOTING_HEADING_DEGREES,
                        "Align X before approaching flower wall",
                        2.0, Math.toRadians(4.0), false);
                if (!reachedPrewallAlignment) {
                    logSecondApproachPollen = false;
                    return;
                }
                // Vision is most repeatable as a separate low-speed lateral
                // alignment after heading and coarse odometry have settled.
                if (!alignToPollenColumn()) {
                    logSecondApproachPollen = false;
                    return;
                }
                double alignedFlowerX = x;
                boolean reachedSecondPosition = driveToPose(alignedFlowerX,
                        SECOND_FINAL_Y_INCHES,
                        SECOND_SHOOTING_HEADING_DEGREES,
                        "Drive and turn into new shooting position",
                        2.25, Math.toRadians(1.5), true,
                        SECOND_FINAL_APPROACH_TIMEOUT_SECONDS);
                logSecondApproachPollen = false;
                if (!reachedSecondPosition) return;

                if (!waitForShooterSpinup()) return;
                if (!runSecondShootingCycleWithIntake()) return;
                runParkPath("Normal park after second shooting");
            }
        } catch (ParkNowException ignored) {
            runParkPath("Five-second emergency park");
        } finally {
            RobotLog.ii(TAG, "RUN_END id=%s last_pose=(%.2f,%.2f,%.1f) state=%s",
                    runId, x, y, Math.toDegrees(heading), shootingState);
            stopDrive();
            stopIntake();
            shooter.setVelocity(0);
            windmill.setPower(0);
            if (limelight != null) limelight.stop();
            shootingState = "STOPPED";
        }

        telemetry.addLine("Complete");
        telemetry.addData("Final X / Y", "%.1f / %.1f in", x, y);
        addShootingTelemetry();
        telemetry.update();
        sleep(1000);
    }

    private void checkParkDeadline() {
        if (!parking && autonomousTimer.seconds() >= PARK_DEADLINE_SECONDS) {
            RobotLog.ww(TAG, "PARK_DEADLINE id=%s elapsed=%.2f pose=(%.2f,%.2f,%.1f)",
                    runId, autonomousTimer.seconds(), x, y, Math.toDegrees(heading));
            throw new ParkNowException();
        }
    }

    private void runParkPath(String reason) {
        if (!opModeIsActive()) return;
        parking = true;
        logSecondApproachPollen = false;
        windmill.setPower(0);
        shooter.setVelocity(0);
        startIntake();
        updatePose();
        shootingState = "PARKING";
        RobotLog.ii(TAG, "PARK_START id=%s reason=%s elapsed=%.2f "
                        + "from=(%.2f,%.2f,%.1f) target=(%.2f,%.2f,%.1f)",
                runId, reason, autonomousTimer.seconds(), x, y,
                Math.toDegrees(heading), PARK_X_INCHES, PARK_Y_INCHES,
                PARK_HEADING_DEGREES);
        driveToPose(PARK_X_INCHES, PARK_Y_INCHES, PARK_HEADING_DEGREES,
                reason, 2.0, Math.toRadians(5.0), true,
                EMERGENCY_PARK_TIMEOUT_SECONDS);
        RobotLog.ii(TAG, "PARK_END id=%s elapsed=%.2f pose=(%.2f,%.2f,%.1f)",
                runId, autonomousTimer.seconds(), x, y, Math.toDegrees(heading));
    }

    private static final class ParkNowException extends RuntimeException {
    }

    private void startShooter(double targetVelocity) {
        shootingState = "SHOOTER SPIN-UP";
        shooterTargetVelocity = targetVelocity;
        shooter.setVelocity(shooterTargetVelocity);
        RobotLog.ii(TAG, "Shooter spin-up started");
    }

    private boolean waitForShooterSpinup() {
        ElapsedTime spinupTimer = new ElapsedTime();
        while (opModeIsActive()
                && Math.abs(shooter.getVelocity()) < SHOOTER_READY_VELOCITY
                && spinupTimer.seconds() < SHOOTER_SPINUP_TIMEOUT_SECONDS) {
            checkParkDeadline();
            shooter.setVelocity(shooterTargetVelocity);
            double velocity = Math.abs(shooter.getVelocity());
            telemetry.addData("Step", "Shooter spin-up");
            telemetry.addData("Shooter", "%.0f / %.0f ticks/s",
                    velocity, shooterTargetVelocity);
            telemetry.addData("Spin-up timeout", "%.1f / %.1f sec",
                    spinupTimer.seconds(), SHOOTER_SPINUP_TIMEOUT_SECONDS);
            addShootingTelemetry();
            telemetry.update();
            idle();
        }
        if (!opModeIsActive()) return false;
        double velocity = Math.abs(shooter.getVelocity());
        if (velocity < SHOOTER_READY_VELOCITY) {
            shootingState = "SHOOTER FAILED TO REACH SPEED";
            RobotLog.ee(TAG, "Shooter only reached %.0f of %.0f ticks/s",
                    velocity, SHOOTER_READY_VELOCITY);
            shooter.setVelocity(0);
            return false;
        }
        RobotLog.ii(TAG, "Shooter ready at %.0f ticks/s after %.1f seconds",
                velocity, spinupTimer.seconds());
        return true;
    }

    private void runShooterAndWindmill() {
        shootingState = "SHOOTER + WINDMILL RUNNING";
        windmill.setPower(-1);
        ElapsedTime shootingTimer = new ElapsedTime();
        RobotLog.ii(TAG, "Windmill running continuously with shooter");
        while (opModeIsActive() && shootingTimer.seconds() < SHOOTING_SECONDS) {
            checkParkDeadline();
            // Hold one velocity for every ball; do not reduce the target late
            // in the cycle while the windmill is still feeding.
            shooterTargetVelocity = SHOOTER_VELOCITY;
            shooter.setVelocity(shooterTargetVelocity);
            telemetry.addData("Step", "Continuous shooting");
            telemetry.addData("Shooting time", "%.1f / %.1f sec",
                    shootingTimer.seconds(), SHOOTING_SECONDS);
            addShootingTelemetry();
            telemetry.update();
            idle();
        }

        windmill.setPower(0);
        shootingState = "SHOOTER RUNNING - DRIVING TO COLLECTION";
        RobotLog.ii(TAG, "Windmill stopped; shooter remains at full speed");
    }

    private boolean runSecondShootingCycleWithIntake() {
        stopDrive();
        shootingState = "SECOND POSITION - INTAKE PRELOAD";
        int pressCenterTarget = readCenter();
        updatePose();
        // Always press at the calibrated flower-contact heading rather than
        // preserving a small arrival error that wall contact can amplify.
        double pressHeadingTarget = Math.toRadians(SECOND_SHOOTING_HEADING_DEGREES);
        pressLogTimer.reset();
        RobotLog.ii(TAG, "PRESS_START id=%s pose=(%.2f,%.2f,%.1f) center=%d heading=%.1f",
                runId, x, y, Math.toDegrees(heading), pressCenterTarget,
                Math.toDegrees(pressHeadingTarget));
        intakeMotor.setPower(1.0);
        leftIntakeServo.setPower(-1.0);
        rightIntakeServo.setPower(-1.0);

        ElapsedTime intakeLeadTimer = new ElapsedTime();
        while (opModeIsActive()
                && intakeLeadTimer.seconds() < SECOND_POSITION_INTAKE_LEAD_SECONDS) {
            checkParkDeadline();
            // Hold a straight, repeatable press so the left intake wheel's
            // contact point can be calibrated against the lower pollen.
            holdStraightIntakePress(pressCenterTarget, pressHeadingTarget,
                    SECOND_POSITION_CONTACT_FORWARD_POWER);
            shooter.setVelocity(SHOOTER_VELOCITY);
            telemetry.addData("Step", "Straight forward intake press");
            telemetry.addData("Intake lead", "%.2f / %.2f sec",
                    intakeLeadTimer.seconds(), SECOND_POSITION_INTAKE_LEAD_SECONDS);
            telemetry.addData("Press forward / turn", "%+.2f / 0.00",
                    SECOND_POSITION_CONTACT_FORWARD_POWER);
            addShootingTelemetry();
            telemetry.update();
            idle();
        }
        if (!opModeIsActive()) {
            stopIntake();
            return false;
        }

        stopDrive();
        shootingState = "SECOND SHOOTING - INTAKE RUNNING";
        windmill.setPower(-1);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < SECOND_SHOOTING_SECONDS) {
            checkParkDeadline();
            // Do not run odometry pose correction here: the powered intake
            // shares the left odometry encoder channel. Keep a simple forward
            // press while intake, shooter, and windmill feed together.
            holdStraightIntakePress(pressCenterTarget, pressHeadingTarget,
                    SECOND_POSITION_HOLD_FORWARD_POWER);
            shooterTargetVelocity = SHOOTER_VELOCITY;
            shooter.setVelocity(shooterTargetVelocity);
            telemetry.addData("Step", "Second shooting + forward intake press");
            telemetry.addData("Shooting time", "%.1f / %.1f sec",
                    timer.seconds(), SECOND_SHOOTING_SECONDS);
            addShootingTelemetry();
            telemetry.update();
            idle();
        }

        windmill.setPower(0);
        stopIntake();
        stopDrive();
        shootingState = "SECOND SHOOTING COMPLETE";
        RobotLog.ii(TAG, "Second shooting cycle with intake complete");
        return opModeIsActive();
    }

    private void holdStraightIntakePress(int centerTargetTicks,
                                         double headingTargetRadians,
                                         double forwardPower) {
        double lateralError = (centerTargetTicks - readCenter())
                / CENTER_TICKS_PER_INCH;
        updatePose();
        double currentHeading = heading;
        double headingError = normalizeRadians(headingTargetRadians - currentHeading);

        double leftCorrection = Range.clip(
                lateralError * SECOND_POSITION_LATERAL_HOLD_KP,
                -SECOND_POSITION_MAX_LATERAL_HOLD_POWER,
                SECOND_POSITION_MAX_LATERAL_HOLD_POWER);
        double turnCorrection = Range.clip(
                headingError * SECOND_POSITION_HEADING_HOLD_KP,
                -SECOND_POSITION_MAX_HEADING_HOLD_POWER,
                SECOND_POSITION_MAX_HEADING_HOLD_POWER);

        if (pressLogTimer.milliseconds() >= 100) {
            pressLogTimer.reset();
            RobotLog.ii(TAG, "PRESS_HOLD id=%s state=%s pose=(%.2f,%.2f,%.1f) "
                            + "center=%d target_center=%d lateral_error=%.3f "
                            + "heading=%.2f target_heading=%.2f heading_error=%.2f "
                            + "cmd=(F=%.2f,L=%.3f,T=%.3f)",
                    runId, shootingState, x, y, Math.toDegrees(heading),
                    readCenter(), centerTargetTicks, lateralError,
                    Math.toDegrees(currentHeading), Math.toDegrees(headingTargetRadians),
                    Math.toDegrees(headingError), forwardPower,
                    leftCorrection, turnCorrection);
        }

        // Drivetrain mounting reverses logical left and CCW commands.
        setMecanum(forwardPower,
                -leftCorrection, -turnCorrection);
        telemetry.addData("Press lateral / heading error", "%.2f in / %+.1f deg",
                lateralError, Math.toDegrees(headingError));
        telemetry.addData("Press correction L / T", "%+.2f / %+.2f",
                leftCorrection, turnCorrection);
    }

    private void recordPostShootingPickupPose() {
        stopDrive();
        shootingState = "READY TO COLLECT - INTAKE + SHOOTER RUNNING";
        intakeMotor.setPower(1.0);
        leftIntakeServo.setPower(-1.0);
        rightIntakeServo.setPower(-1.0);

        // The intake motor shares the left odometry encoder channel. Keep the
        // intended collection mechanism running until the operator explicitly
        // enters safe recording mode, then discard those intake encoder ticks.
        while (opModeIsActive() && !gamepad1.a) {
            telemetry.addData("Mode", "POST-SHOOTING COLLECTION TEST");
            telemetry.addLine("Intake and shooter are running; parking is disabled");
            telemetry.addLine("Press A to stop mechanisms and record a pushed pose");
            addShootingTelemetry();
            telemetry.update();
            idle();
        }
        if (!opModeIsActive()) return;

        stopIntake();
        shooter.setVelocity(0);
        shootingState = "SAFE PUSH-POSITION RECORDER";
        syncPodBaselines();
        while (opModeIsActive() && gamepad1.a) idle();

        while (opModeIsActive() && !gamepad1.b) {
            updatePose();
            stopDrive();
            telemetry.addData("Mode", "PUSH ROBOT TO NEW PICKUP POSITION");
            telemetry.addData("X forward", "%+.2f in", x);
            telemetry.addData("Y left", "%+.2f in", y);
            telemetry.addData("Heading CCW", "%+.1f deg", Math.toDegrees(heading));
            telemetry.addLine("Press B to capture this pose");
            telemetry.update();
            idle();
        }
        if (!opModeIsActive()) return;

        updatePose();
        RobotLog.ii(TAG, "POST-SHOOTING PICKUP POSE x=%.2f y=%.2f heading=%.1f",
                x, y, Math.toDegrees(heading));
        while (opModeIsActive()) {
            stopDrive();
            telemetry.addData("Mode", "PICKUP POSE CAPTURED");
            telemetry.addData("RECORDED X forward", "%+.2f in", x);
            telemetry.addData("RECORDED Y left", "%+.2f in", y);
            telemetry.addData("RECORDED heading CCW", "%+.1f deg",
                    Math.toDegrees(heading));
            telemetry.update();
            idle();
        }
    }

    private void holdSecondShootingPose() {
        updatePose();
        double targetHeading = Math.toRadians(SECOND_SHOOTING_HEADING_DEGREES);
        double errorX = SECOND_FINAL_X_INCHES - x;
        double errorY = SECOND_FINAL_Y_INCHES - y;
        double positionError = Math.hypot(errorX, errorY);
        double headingError = normalizeRadians(targetHeading - heading);

        double forward = 0;
        double left = 0;
        if (positionError > POSITION_TOLERANCE_INCHES) {
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double robotForwardError = errorX * cos + errorY * sin;
            double robotLeftError = -errorX * sin + errorY * cos;
            if (Math.abs(robotForwardError) > 0.75) {
                forward = minimumPower(
                        Range.clip(robotForwardError * TRANSLATION_KP, -0.25, 0.25),
                        MIN_FORWARD_POWER, robotForwardError);
            }
            if (Math.abs(robotLeftError) > 0.75) {
                left = minimumPower(
                        Range.clip(robotLeftError * TRANSLATION_KP, -0.30, 0.30),
                        MIN_STRAFE_POWER, robotLeftError);
            }
        }

        double turn = 0;
        if (Math.abs(headingError) > HEADING_TOLERANCE_RADIANS) {
            turn = Range.clip(headingError * HEADING_KP, -0.30, 0.30);
            if (Math.abs(turn) < MIN_TURN_POWER) {
                turn = Math.copySign(MIN_TURN_POWER, turn);
            }
        }

        // Map pose-controller commands to the updated drivetrain mounting.
        setMecanum(forward, -left, -turn);
        telemetry.addData("Shooting pose error", "%.2f in / %+.1f deg",
                positionError, Math.toDegrees(headingError));
        telemetry.addData("Pose correction", "%s",
                forward == 0 && left == 0 && turn == 0 ? "HOLDING" : "CORRECTING");
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
                    -gamepad1.left_stick_y * 0.40,
                    gamepad1.left_stick_x * 0.40,
                    gamepad1.right_stick_x * 0.35);

            telemetry.addData("Mode", "NEW INTAKE + SHOOTING POSITION");
            telemetry.addData("X forward", "%+.2f in", x);
            telemetry.addData("Y left", "%+.2f in", y);
            telemetry.addData("Heading CCW", "%+.1f deg", Math.toDegrees(heading));
            telemetry.addLine("Gamepad 1 sticks move robot; press B to capture");
            telemetry.addLine("No software timeout - waits until B or Stop");
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
        // This is a look-ahead waypoint, not a stopping target. Hand off to the
        // next target while the robot still has useful speed so the route forms
        // one continuous curve instead of slowing and re-accelerating here.
        return driveToPose(targetX, targetY, targetHeadingDegrees, step,
                18.0, Math.toRadians(35.0), false);
    }

    private boolean driveForwardWithIntake(double distanceInches) {
        int startingRightTicks = readRight();
        ElapsedTime timer = new ElapsedTime();
        double distanceTraveled = 0.0;
        boolean targetReached = false;
        intakeMotor.setPower(1.0);
        leftIntakeServo.setPower(-1.0);
        rightIntakeServo.setPower(-1.0);

        while (opModeIsActive()
                && timer.seconds() < INTAKE_FORWARD_TIMEOUT_SECONDS) {
            checkParkDeadline();
            distanceTraveled =
                    (readRight() - startingRightTicks) / RIGHT_TICKS_PER_INCH;
            if (distanceTraveled
                    >= distanceInches - INTAKE_FORWARD_TOLERANCE_INCHES) {
                targetReached = true;
                break;
            }

            setMecanum(INTAKE_FORWARD_POWER, 0, 0);
            telemetry.addData("Step", "Forward %.0f inches with intake", distanceInches);
            telemetry.addData("Distance", "%.1f / %.1f in",
                    distanceTraveled, distanceInches);
            telemetry.addData("Intake drive time", "%.1f / %.1f sec",
                    timer.seconds(), INTAKE_FORWARD_TIMEOUT_SECONDS);
            telemetry.update();
            idle();
        }

        stopDrive();
        if (!opModeIsActive()) {
            stopIntake();
            return false;
        }

        // A timeout no longer cancels the rest of autonomous. Continue using
        // the measured distance so the following pose targets remain accurate.
        distanceTraveled =
                (readRight() - startingRightTicks) / RIGHT_TICKS_PER_INCH;
        if (!targetReached) {
            RobotLog.ww(TAG,
                    "Intake approach timed out at %.1f / %.1f inches; continuing",
                    distanceTraveled, distanceInches);
        }

        ElapsedTime intakeTimer = new ElapsedTime();
        while (opModeIsActive()
                && intakeTimer.seconds() < INTAKE_AFTER_DRIVE_SECONDS) {
            checkParkDeadline();
            telemetry.addData("Step", targetReached
                    ? "Stopped; intake still running"
                    : "Intake drive timed out; continuing after hold");
            telemetry.addData("Distance", "%.1f / %.1f in",
                    distanceTraveled, distanceInches);
            telemetry.addData("Intake time", "%.1f / %.1f sec",
                    intakeTimer.seconds(), INTAKE_AFTER_DRIVE_SECONDS);
            telemetry.update();
            idle();
        }

        // Keep all intake components running through the entire second
        // approach. Its motor now has a separate encoder port from all three
        // odometry pods, so normal three-pod localization remains valid.
        x += distanceTraveled * Math.cos(heading);
        y += distanceTraveled * Math.sin(heading);
        syncPodBaselines();
        RobotLog.ii(TAG, "INTAKE_CONTINUOUS id=%s pose=(%.2f,%.2f,%.1f) "
                        + "using=left+right+center",
                runId, x, y, Math.toDegrees(heading));
        return opModeIsActive();
    }

    private void stopIntake() {
        intakeMotor.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }

    private void startIntake() {
        intakeMotor.setPower(1.0);
        leftIntakeServo.setPower(-1.0);
        rightIntakeServo.setPower(-1.0);
    }

    private boolean alignToPollenColumn() {
        final double targetHeading = Math.toRadians(SECOND_SHOOTING_HEADING_DEGREES);
        final double holdY = y;
        ElapsedTime timer = new ElapsedTime();
        long processedDetectionNanos = 0;
        int stableFrames = 0;
        double filteredTx = Double.NaN;

        latestPollenColumnTx = Double.NaN;
        latestPollenColumnNanos = 0;
        limelightLogTimer.reset();
        RobotLog.ii(TAG, "VISION_ALIGN_START id=%s pose=(%.2f,%.2f,%.1f) target_tx=%.2f",
                runId, x, y, Math.toDegrees(heading), POLLEN_COLUMN_TARGET_TX_DEGREES);

        while (opModeIsActive() && timer.seconds() < POLLEN_ALIGNMENT_TIMEOUT_SECONDS) {
            checkParkDeadline();
            startIntake();
            updatePose();
            logSecondApproachPollen();

            double headingError = normalizeRadians(targetHeading - heading);
            boolean newColumn = latestPollenColumnNanos != 0
                    && latestPollenColumnNanos != processedDetectionNanos;
            if (newColumn) {
                processedDetectionNanos = latestPollenColumnNanos;
                filteredTx = Double.isNaN(filteredTx)
                        ? latestPollenColumnTx
                        : 0.5 * latestPollenColumnTx + 0.5 * filteredTx;
                if (Math.abs(headingError) <= Math.toRadians(4.0)
                        && Math.abs(latestPollenColumnTx - POLLEN_COLUMN_TARGET_TX_DEGREES)
                        <= POLLEN_ALIGNMENT_TOLERANCE_DEGREES) {
                    stableFrames++;
                } else {
                    stableFrames = 0;
                }
            }

            if (stableFrames >= POLLEN_ALIGNMENT_STABLE_FRAMES) {
                stopDrive();
                RobotLog.ii(TAG, "VISION_ALIGN_LOCK id=%s elapsed=%.2f "
                                + "filtered_tx=%.2f frames=%d pose=(%.2f,%.2f,%.1f)",
                        runId, timer.seconds(), filteredTx, stableFrames,
                        x, y, Math.toDegrees(heading));
                return true;
            }

            double txError = Double.isNaN(filteredTx)
                    ? 0 : filteredTx - POLLEN_COLUMN_TARGET_TX_DEGREES;
            double robotLeft = 0;
            if (!Double.isNaN(filteredTx) && pollenColumnRecentlyVisible()
                    && Math.abs(headingError) <= Math.toRadians(6.0)) {
                // Run data shows this camera mounting mirrors horizontal tx:
                // negative tx occurred while the robot was physically left of
                // the flower. Therefore negative error must command robot-right.
                robotLeft = Range.clip(txError * POLLEN_COLUMN_STRAFE_KP,
                        -POLLEN_COLUMN_MAX_STRAFE, POLLEN_COLUMN_MAX_STRAFE);
                if (Math.abs(txError) > POLLEN_ALIGNMENT_TOLERANCE_DEGREES
                        && Math.abs(robotLeft) < POLLEN_COLUMN_MIN_STRAFE) {
                    robotLeft = Math.copySign(POLLEN_COLUMN_MIN_STRAFE, robotLeft);
                }
            }

            double errorY = holdY - y;
            double forward = Range.clip(errorY * Math.sin(heading) * TRANSLATION_KP,
                    -0.12, 0.12);
            double turn = Range.clip(headingError * HEADING_KP, -0.20, 0.20);
            setMecanum(forward, -robotLeft, -turn);

            telemetry.addData("Step", "Slow Limelight flower alignment");
            telemetry.addData("Column raw / filtered / target", "%+.2f / %+.2f / %+.2f",
                    latestPollenColumnTx, filteredTx, POLLEN_COLUMN_TARGET_TX_DEGREES);
            telemetry.addData("Stable frames", "%d / %d",
                    stableFrames, POLLEN_ALIGNMENT_STABLE_FRAMES);
            telemetry.addData("Vision strafe", "%+.2f", robotLeft);
            telemetry.update();
            idle();
        }

        stopDrive();
        if (!opModeIsActive()) return false;
        RobotLog.ww(TAG, "VISION_ALIGN_TIMEOUT id=%s filtered_tx=%s frames=%d "
                        + "locking_odom_x=%.2f pose=(%.2f,%.2f,%.1f)",
                runId, Double.isNaN(filteredTx) ? "none"
                        : String.format(Locale.US, "%.2f", filteredTx),
                stableFrames, x, x, y, Math.toDegrees(heading));
        return true;
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
        return driveToPose(targetX, targetY, targetHeadingDegrees, step,
                positionTolerance, headingTolerance, stopAtTarget,
                WAYPOINT_TIMEOUT_SECONDS);
    }

    private boolean driveToPose(double targetX, double targetY,
                                double targetHeadingDegrees, String step,
                                double positionTolerance, double headingTolerance,
                                boolean stopAtTarget, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        double targetHeading = Math.toRadians(targetHeadingDegrees);
        RobotLog.ii(TAG, "WAYPOINT_START id=%s step=%s target=(%.2f,%.2f,%.1f) "
                        + "start=(%.2f,%.2f,%.1f) timeout=%.1f",
                runId, step, targetX, targetY, targetHeadingDegrees,
                x, y, Math.toDegrees(heading), timeoutSeconds);

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            checkParkDeadline();
            // Reassert the continuous intake command throughout every path.
            startIntake();
            updatePose();
            if (logSecondApproachPollen) logSecondApproachPollen();
            double errorX = targetX - x;
            double errorY = targetY - y;
            double positionError = Math.hypot(errorX, errorY);
            double headingError = normalizeRadians(targetHeading - heading);

            if (positionError <= positionTolerance
                    && Math.abs(headingError) <= headingTolerance) {
                if (stopAtTarget) stopDrive();
                RobotLog.ii(TAG, "WAYPOINT_REACHED id=%s step=%s elapsed=%.2f "
                                + "pose=(%.2f,%.2f,%.1f) error=(%.2f,%.1f)",
                        runId, step, timer.seconds(), x, y, Math.toDegrees(heading),
                        positionError, Math.toDegrees(headingError));
                return true;
            }

            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double robotForwardError = errorX * cos + errorY * sin;
            double robotLeftError = -errorX * sin + errorY * cos;
            // Each axis must remain below this value for the combined radial
            // error to fit inside the requested position tolerance. The old
            // fixed 0.75-inch dead zone made a 0.60-inch tolerance impossible.
            double axisTolerance = positionTolerance / Math.sqrt(2.0);

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

            if (logSecondApproachPollen
                    && Math.abs(headingError)
                    > SECOND_APPROACH_LARGE_HEADING_ERROR_RADIANS) {
                // Rotate early enough to expose the flower to the Limelight,
                // while continuing along the same curved odometry path.
                forward *= SECOND_APPROACH_TRANSLATION_SCALE_WHILE_TURNING;
                left *= SECOND_APPROACH_TRANSLATION_SCALE_WHILE_TURNING;
                turn = Range.clip(headingError * SECOND_APPROACH_TURN_KP,
                        -SECOND_APPROACH_MAX_TURN_POWER,
                        SECOND_APPROACH_MAX_TURN_POWER);
            }

            if (Math.abs(robotForwardError) <= axisTolerance) forward = 0;
            if (Math.abs(robotLeftError) <= axisTolerance) left = 0;
            if (Math.abs(headingError) <= headingTolerance) {
                turn = 0;
            } else if (Math.abs(turn) < MIN_TURN_POWER) {
                turn = Math.copySign(MIN_TURN_POWER, turn);
            }

            setMecanum(forward, -left, -turn);

            telemetry.addData("Step", step);
            telemetry.addData("Target X / Y", "%.1f / %.1f in", targetX, targetY);
            telemetry.addData("Target heading", "%.0f deg", targetHeadingDegrees);
            telemetry.addData("Pose X / Y", "%+.1f / %+.1f in", x, y);
            telemetry.addData("Heading", "%+.1f deg", Math.toDegrees(heading));
            telemetry.addData("Position error", "%.2f in", positionError);
            telemetry.addData("Heading error", "%+.1f deg",
                    Math.toDegrees(headingError));
            telemetry.addData("Waypoint time", "%.1f / %.1f sec",
                    timer.seconds(), timeoutSeconds);
            telemetry.addData("Drive command F / L / T", "%+.2f / %+.2f / %+.2f",
                    forward, left, turn);
            if (logSecondApproachPollen) {
                telemetry.addData("Column tx", "%+.2f", latestPollenColumnTx);
            }
            addShootingTelemetry();
            telemetry.update();
            idle();
        }

        stopDrive();
        if (!opModeIsActive()) return false;

        // Missing a tight pose tolerance should not silently cancel every
        // remaining autonomous action. Stop safely, report the miss, and let
        // the next step continue from the measured pose.
        RobotLog.ww(TAG, "WAYPOINT_TIMEOUT id=%s step=%s pose=(%.2f,%.2f,%.1f) "
                        + "target=(%.2f,%.2f,%.1f)",
                runId, step, x, y, Math.toDegrees(heading),
                targetX, targetY, targetHeadingDegrees);
        telemetry.addData("Status", "Timed out during %s; continuing", step);
        addShootingTelemetry();
        telemetry.update();
        return true;
    }

    private void logSecondApproachPollen() {
        if (limelightLogTimer.milliseconds() < 100) return;
        limelightLogTimer.reset();

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.DetectorResult> pollenDetections = validPollen(result);
        if (pollenDetections.isEmpty()) {
            RobotLog.dd(TAG, "SECOND_APPROACH_POLLEN id=%s none staleness_ms=%d pose=(%.2f,%.2f,%.1f)",
                    runId,
                    result == null ? -1 : result.getStaleness(),
                    x, y, Math.toDegrees(heading));
            telemetry.addData("Second-approach pollen", "not detected");
            return;
        }

        double columnTx = findPollenColumnTx(pollenDetections);
        if (!Double.isNaN(columnTx)) {
            latestPollenColumnTx = columnTx;
            latestPollenColumnNanos = System.nanoTime();
        }

        LLResultTypes.DetectorResult pollen = pollenDetections.get(0);
        double minTx = Double.POSITIVE_INFINITY;
        double maxTx = Double.NEGATIVE_INFINITY;
        double minTy = Double.POSITIVE_INFINITY;
        double maxTy = Double.NEGATIVE_INFINITY;
        StringBuilder detections = new StringBuilder();
        for (int i = 0; i < pollenDetections.size(); i++) {
            LLResultTypes.DetectorResult detection = pollenDetections.get(i);
            double detectionTx = detection.getTargetXDegrees();
            double detectionTy = detection.getTargetYDegrees();
            minTx = Math.min(minTx, detectionTx);
            maxTx = Math.max(maxTx, detectionTx);
            minTy = Math.min(minTy, detectionTy);
            maxTy = Math.max(maxTy, detectionTy);
            if (i > 0) detections.append(';');
            detections.append(String.format(Locale.US,
                    "#%d(tx=%+.2f,ty=%+.2f,area=%.3f,conf=%.2f)",
                    i, detectionTx, detectionTy, detection.getTargetArea(),
                    detection.getConfidence()));
        }

        double area = pollen.getTargetArea();
        double tx = pollen.getTargetXDegrees();
        double ty = pollen.getTargetYDegrees();
        double estimatedForward = LIMELIGHT_DISTANCE_SCALE
                / Math.sqrt(Math.max(area, LIMELIGHT_MIN_AREA_PERCENT));
        double estimatedRight = LIMELIGHT_CAMERA_RIGHT_INCHES
                + estimatedForward * Math.tan(Math.toRadians(tx));

        RobotLog.ii(TAG, String.format(Locale.US,
                "SECOND_APPROACH_POLLEN id=%s count=%d tx_span=%.2f ty_span=%.2f "
                        + "column_tx=%s column_error=%s forward=%.2f right=%.2f "
                        + "tx=%.2f ty=%.2f area=%.4f "
                        + "confidence=%.3f pose=(%.2f,%.2f,%.1f) detections=%s",
                runId, pollenDetections.size(), maxTx - minTx, maxTy - minTy,
                Double.isNaN(columnTx) ? "none" : String.format(Locale.US, "%+.2f", columnTx),
                Double.isNaN(columnTx) ? "none" : String.format(Locale.US, "%+.2f",
                        columnTx - POLLEN_COLUMN_TARGET_TX_DEGREES),
                estimatedForward, estimatedRight, tx, ty, area,
                pollen.getConfidence(), x, y, Math.toDegrees(heading), detections));
        telemetry.addData("Pollen detections / tx span / ty span", "%d / %.1f / %.1f",
                pollenDetections.size(), maxTx - minTx, maxTy - minTy);
        telemetry.addData("Pollen relative F / R", "%.1f / %+.1f in",
                estimatedForward, estimatedRight);
        telemetry.addData("Pollen tx / ty", "%+.1f / %+.1f deg", tx, ty);
        telemetry.addData("Pollen column tx", Double.isNaN(columnTx)
                ? "not confirmed" : String.format(Locale.US, "%+.2f deg", columnTx));
    }

    private double findPollenColumnTx(
            List<LLResultTypes.DetectorResult> detections) {
        double bestScore = Double.NEGATIVE_INFINITY;
        double bestTx = Double.NaN;
        for (int i = 0; i < detections.size(); i++) {
                LLResultTypes.DetectorResult a = detections.get(i);
            for (int j = i + 1; j < detections.size(); j++) {
                LLResultTypes.DetectorResult b = detections.get(j);
                if (a.getTargetArea() < POLLEN_COLUMN_MIN_MEMBER_AREA_PERCENT
                        || b.getTargetArea() < POLLEN_COLUMN_MIN_MEMBER_AREA_PERCENT) {
                    continue;
                }
                double txSpread = Math.abs(a.getTargetXDegrees()
                        - b.getTargetXDegrees());
                double tySpread = Math.abs(a.getTargetYDegrees()
                        - b.getTargetYDegrees());
                if (txSpread > POLLEN_COLUMN_MAX_TX_SPREAD_DEGREES
                        || tySpread < POLLEN_COLUMN_MIN_TY_SPREAD_DEGREES
                        || tySpread > POLLEN_COLUMN_MAX_TY_SPREAD_DEGREES) {
                    continue;
                }
                double pairTx = (a.getTargetXDegrees()
                        + b.getTargetXDegrees()) / 2.0;
                // The real flower column has consistently appeared near the
                // center of the camera. Far-edge pairs at the 20% threshold
                // are wall/background false positives and must remain log-only.
                if (pairTx < POLLEN_COLUMN_MIN_GUIDANCE_TX_DEGREES
                        || pairTx > POLLEN_COLUMN_MAX_GUIDANCE_TX_DEGREES) {
                    continue;
                }
                double score = a.getConfidence() + b.getConfidence();
                if (score > bestScore) {
                    bestScore = score;
                    bestTx = pairTx;
                }
            }
        }
        return bestTx;
    }

    private boolean pollenColumnRecentlyVisible() {
        return latestPollenColumnNanos != 0
                && (System.nanoTime() - latestPollenColumnNanos) / 1_000_000L
                <= POLLEN_COLUMN_HOLD_MS;
    }

    private LLResultTypes.DetectorResult closestPollen(LLResult result) {
        List<LLResultTypes.DetectorResult> detections = validPollen(result);
        return detections.isEmpty() ? null : detections.get(0);
    }

    private List<LLResultTypes.DetectorResult> validPollen(LLResult result) {
        List<LLResultTypes.DetectorResult> pollen = new ArrayList<>();
        if (result == null || result.getStaleness() > LIMELIGHT_MAX_STALENESS_MS) {
            return pollen;
        }
        for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {
            String label = detection.getClassName();
            if (label == null
                    || !label.toLowerCase(Locale.US).trim().equals("pollen")
                    || detection.getConfidence() < LIMELIGHT_MIN_CONFIDENCE
                    || detection.getTargetArea() < LIMELIGHT_MIN_AREA_PERCENT) {
                continue;
            }
            pollen.add(detection);
        }
        pollen.sort((a, b) -> Double.compare(b.getTargetArea(), a.getTargetArea()));
        return pollen;
    }

    private void addShootingTelemetry() {
        telemetry.addData("Shooting state", shootingState);
        telemetry.addData("Actual shooter speed", "%.1f ticks/s",
                Math.abs(shooter.getVelocity()));
        telemetry.addData("Shooter target", "%.0f ticks/s", shooterTargetVelocity);
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
        double deltaRight = (currentRight - previousRight) / RIGHT_TICKS_PER_INCH;
        double deltaCenter = (currentCenter - previousCenter) / CENTER_TICKS_PER_INCH;
        double deltaLeft = (currentLeft - previousLeft) / LEFT_TICKS_PER_INCH;
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
        syncPodBaselines();
        x = 0;
        y = 0;
        heading = 0;
    }

    private void syncPodBaselines() {
        previousLeft = readLeft();
        previousRight = readRight();
        previousCenter = readCenter();
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

    // frontLeft is configured REVERSE, which also reverses its encoder count.
    // That produces the same forward-positive sign as the former -raw reading.
    private int readLeft() { return leftPod.getCurrentPosition(); }
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
