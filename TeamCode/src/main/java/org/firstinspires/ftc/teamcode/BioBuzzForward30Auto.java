package org.firstinspires.ftc.teamcode;

import android.util.Size;

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
import com.qualcomm.robotcore.hardware.Servo;
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

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Drives one continuous curved path to x=47, y=-67, heading=-95 degrees. */
@Autonomous(name = "Red Autonomous Mode", group = "Jimothy")
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
    private static final double LEFT_TICKS_PER_INCH = 503.47;
    private static final double RIGHT_TICKS_PER_INCH = 504.53;
    private static final double CENTER_TICKS_PER_INCH = 510.30;
    private static final double POD_SPACING_INCHES = 12.10;
    private static final double CENTER_POD_X_INCHES = -6.56;

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
    // Tune the two autonomous shooting positions independently.
    private static final double FIRST_SHOOTER_SPEED_FRACTION = 0.48;
    private static final double SECOND_SHOOTER_SPEED_FRACTION = 0.47;
    private static final double SHOOTER_READY_FRACTION_OF_TARGET = 0.97;
    private static final double SHOOTER_SPINUP_TIMEOUT_SECONDS = 3.0;
    private static final double SHOOTING_SECONDS = 4.0;
    private static final double SECOND_SHOT_TAG_TIMEOUT_SECONDS = 2.0;
    // Measured from the audience-side shooting origin during the pushed
    // calibration run. Keep the robot in its starting orientation.
    private static final double RED_INTAKE_WAYPOINT_1_X_INCHES = -6.44;
    private static final double RED_INTAKE_WAYPOINT_1_Y_INCHES = -12.97;
    private static final double RED_INTAKE_WAYPOINT_2_X_INCHES = -5.71;
    private static final double RED_INTAKE_WAYPOINT_2_Y_INCHES = -28.61;
    private static final double RED_INTAKE_WAYPOINT_3_X_INCHES = -6.93;
    private static final double RED_INTAKE_WAYPOINT_3_Y_INCHES = -36.70;
    private static final double RED_INTAKE_X_INCHES = -9.70;
    private static final double RED_INTAKE_Y_INCHES = -47.38;
    private static final double RED_INTAKE_HEADING_DEGREES = 0.0;
    private static final double RED_INTAKE_FINAL_HEADING_DEGREES = 2.7;
    private static final double RED_INTAKE_POSITION_TIMEOUT_SECONDS = 2.0;
    private static final double ARM_DOWN_POSITION = 0.05;
    private static final double ARM_UP_POSITION = 0.5;
    private static final long ARM_LOWER_SETTLE_MS = 250;
    private static final long ARM_RAISE_SETTLE_MS = 1000;
    private static final double INTAKE_NUDGE_DISTANCE_INCHES = 1.0;
    private static final double SECOND_INTAKE_REPOSITION_DISTANCE_INCHES = 3.0;
    private static final long SECOND_INTAKE_START_DELAY_MS = 2000;
    private static final double INTAKE_NUDGE_POWER = 0.30;
    private static final double FAST_INTAKE_NUDGE_POWER = 0.50;
    private static final double INTAKE_NUDGE_TIMEOUT_SECONDS = 1.0;
    private static final double SECOND_INTAKE_X_INCHES = -121.07;
    private static final double SECOND_INTAKE_Y_INCHES = -15.38;
    private static final double SECOND_INTAKE_HEADING_DEGREES = -170.0;
    private static final double SECOND_SHOOTING_RIGHT_OFFSET_INCHES = 2.0;
    private static final double SECOND_SHOOTING_X_INCHES = -120.80;
    private static final double SECOND_SHOOTING_Y_INCHES = -0.90;
    private static final double SECOND_SHOOTING_CAPTURED_HEADING_DEGREES = -180.0;
    private static final double SECOND_SHOOTING_POSE_TIMEOUT_SECONDS = 4.0;
    private static final double SECOND_SHOOTING_HEADING_TIMEOUT_SECONDS = 2.0;
    // Fresh high-speed curve from the intake area; these are look-ahead points,
    // not stops, and do not replay the hand-pushed recorded route.
    private static final double FAST_SECOND_PATH_WAYPOINT_1_X = -50.58;
    private static final double FAST_SECOND_PATH_WAYPOINT_1_Y = -33.65;
    private static final double FAST_SECOND_PATH_WAYPOINT_1_HEADING = 2.6;
    private static final double FAST_SECOND_PATH_WAYPOINT_2_X = -94.83;
    private static final double FAST_SECOND_PATH_WAYPOINT_2_Y = -18.08;
    private static final double FAST_SECOND_PATH_WAYPOINT_2_HEADING = 130.5;
    private static final double AFTER_SHOOTING_HEADING_DEGREES = -86.4;
    private static final double BACK_AFTER_SHOOTING_INCHES = 6.0;
    // Restored original left-side collection approach after the first shot.
    private static final double AFTER_SHOOTING_X_INCHES = 2.67;
    private static final double AFTER_SHOOTING_Y_INCHES = -71.16;
    private static final double INTAKE_APPROACH_RIGHT_INCHES = 3.0;
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
    private static final double SECOND_SHOOTING_HEADING_DEGREES = -180.0;
    private static final double SECOND_POSITION_INTAKE_LEAD_SECONDS = 1.0;
    private static final double SECOND_POSITION_CONTACT_FORWARD_POWER = 0.16;
    private static final double SECOND_POSITION_HOLD_FORWARD_POWER = 0.06;
    private static final double SECOND_POSITION_LATERAL_HOLD_KP = 0.04;
    private static final double SECOND_POSITION_MAX_LATERAL_HOLD_POWER = 0.08;
    private static final double SECOND_POSITION_HEADING_HOLD_KP = 1.50;
    private static final double SECOND_POSITION_MAX_HEADING_HOLD_POWER = 0.35;
    private static final double SECOND_SHOOTING_SECONDS = 6.0;
    private static final double WINDMILL_FEED_POWER = -0.20;
    private static final double FINAL_PARK_X_INCHES = -109.93;
    private static final double FINAL_PARK_Y_INCHES = -47.73;
    private static final double FINAL_PARK_HEADING_DEGREES = 90.0;
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
    private Servo flowerArm;
    private Limelight3A limelight;
    private AprilTagProcessor aprilTags;
    private VisionPortal webcamPortal;
    private final ElapsedTime limelightLogTimer = new ElapsedTime();
    private final ElapsedTime pressLogTimer = new ElapsedTime();
    private final ElapsedTime autonomousTimer = new ElapsedTime();
    private static final double FORCED_PARK_TIME_SECONDS = 25.0;
    private boolean forcedParkInProgress;
    private boolean runIntakeDuringPath = true;
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
    private double shooterVelocity;
    private double secondShooterVelocity;
    private double shooterReadyVelocity;
    private double activeShooterReadyVelocity;
    private double shooterTargetVelocity;
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
        double shooterMaxVelocity = shooter.getMotorType().getMaxRPM()
                * shooter.getMotorType().getTicksPerRev() / 60.0;
        shooterVelocity = FIRST_SHOOTER_SPEED_FRACTION * shooterMaxVelocity;
        secondShooterVelocity = SECOND_SHOOTER_SPEED_FRACTION * shooterMaxVelocity;
        shooterReadyVelocity = SHOOTER_READY_FRACTION_OF_TARGET * shooterVelocity;
        activeShooterReadyVelocity = shooterReadyVelocity;
        shooterTargetVelocity = shooterVelocity;
        windmill = hardwareMap.get(CRServo.class, "windmill_servo");
        flowerArm = hardwareMap.get(Servo.class, "flower_arm");
        flowerArm.setDirection(Servo.Direction.REVERSE);
        // Limelight is optional; autonomous continues with odometry and the USB
        // webcam when the Limelight is disconnected or absent from the config.
        limelight = hardwareMap.tryGet(Limelight3A.class, "limelight");
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
        if (limelight != null) {
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();
        } else {
            telemetry.addLine("Limelight disconnected; using odometry/webcam only.");
        }
        aprilTags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .build();
        aprilTags.setDecimation(1.0f);
        webcamPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTags)
                .build();
        stopDrive();
        resetPose();

        telemetry.addLine("Red Auto: start pose is X=0, Y=0, heading=0");
        telemetry.addLine("Shoots 3.5 seconds, then push robot and press B to record");
        addShootingTelemetry();
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        runId = Long.toString(System.currentTimeMillis());
        resetPose();
        autonomousTimer.reset();
        RobotLog.ii(TAG, "RUN_START id=%s target2=(%.2f,%.2f,%.1f)",
                runId, SECOND_FINAL_X_INCHES, SECOND_FINAL_Y_INCHES,
                SECOND_SHOOTING_HEADING_DEGREES);
        // Spin up only for the opening shot. The shooter is forced off again
        // throughout intake position 1 and the drive to intake position 2.
        startShooter(shooterVelocity);
        try {
            RobotLog.ii(TAG, "Opening shot at starting pose; verifying shooter speed");
            telemetry.addLine("Opening shot at starting pose; verifying shooter speed");
            addShootingTelemetry();
            telemetry.update();
            if (!waitForShooterSpinup()) return;
            runShooterAndWindmill();
            shooter.setVelocity(0);
            if (opModeIsActive()) {
                keepShooterOffBeforeSecondIntake();
                startIntake();
                RobotLog.ii(TAG, "INTAKE_START id=%s after_opening_shot=true", runId);
                shootingState = "INTAKE RUNNING - FOLLOWING RECORDED PATH";
                if (!driveToPose(RED_INTAKE_WAYPOINT_1_X_INCHES,
                        RED_INTAKE_WAYPOINT_1_Y_INCHES,
                        RED_INTAKE_HEADING_DEGREES,
                        "Recorded intake path waypoint 1",
                        3.0, Math.toRadians(8.0), false)) return;
                if (!driveToPose(RED_INTAKE_WAYPOINT_2_X_INCHES,
                        RED_INTAKE_WAYPOINT_2_Y_INCHES,
                        RED_INTAKE_HEADING_DEGREES,
                        "Recorded intake path waypoint 2",
                        3.0, Math.toRadians(8.0), false)) return;
                if (!driveToPose(RED_INTAKE_WAYPOINT_3_X_INCHES,
                        RED_INTAKE_WAYPOINT_3_Y_INCHES,
                        RED_INTAKE_HEADING_DEGREES,
                        "Recorded intake path waypoint 3",
                        2.0, Math.toRadians(6.0), false)) return;
                if (!driveToPose(RED_INTAKE_X_INCHES,
                        RED_INTAKE_Y_INCHES,
                        RED_INTAKE_FINAL_HEADING_DEGREES,
                        "Recorded red intake position",
                        POSITION_TOLERANCE_INCHES,
                        HEADING_TOLERANCE_RADIANS,
                        true,
                        RED_INTAKE_POSITION_TIMEOUT_SECONDS)) return;
                beginLoweringArmForIntake();
                finishLoweringArmForIntake();
                if (!driveIntakeNudge(12.0, 0.20)) return;
                keepShooterOffBeforeSecondIntake();
                shootingState = "RED INTAKE COLLECTION COMPLETE";
                // Keep collecting from intake position 1 through the approach
                // to intake position 2, but wait until the second shooting
                // position before spinning the shooter again.
                runIntakeDuringPath = false;

                // Clear the first intake mechanism, then use a dedicated turn.
                // Do not ask the pose controller to hold X/Y while rotating;
                // odometry drift made those corrections fight the turn.
                if (!driveRobotRelative(-5.0, 6.0, 0.0,
                        "Fast clearance: back 5 and left 6 inches",
                        true, 2.0)) return;
                if (!turnInPlaceToHeading(-180.0,
                        "Fast clearance: turn CCW to -180", 3.0)) return;

                updatePose();
                double distanceToSecondIntake = Math.hypot(
                        SECOND_INTAKE_X_INCHES - x,
                        SECOND_INTAKE_Y_INCHES - y);
                double straightDistance = distanceToSecondIntake * (2.0 / 3.0);
                double straightTargetX = x - straightDistance;
                double straightTargetY = y;
                if (!followLineSegment(straightTargetX, straightTargetY,
                        SECOND_INTAKE_HEADING_DEGREES,
                        "Segment: straight first 2/3 to second intake",
                        8.0)) return;
                keepShooterOffBeforeSecondIntake();

                if (!followLineSegment(SECOND_INTAKE_X_INCHES,
                        SECOND_INTAKE_Y_INCHES,
                        SECOND_INTAKE_HEADING_DEGREES,
                        "Segment: diagonal to second intake position", 6.0,
                        0.25)) return;

                // Shift two inches robot-right for the second shot. The later
                // one-inch-left flower approach leaves the final collection
                // and shooting lane net one inch right of the original lane.
                startShooter(secondShooterVelocity);
                stopIntake();
                if (!driveRobotRelative(0.0, -SECOND_SHOOTING_RIGHT_OFFSET_INCHES,
                        SECOND_INTAKE_HEADING_DEGREES,
                        "Shift right 2 inches to second shooting position",
                        true, 2.0)) return;
                if (!waitForSecondShotTarget()) {
                    RobotLog.ww(TAG,
                            "SECOND_SHOT_SKIPPED id=%s target=%s reason=tag_not_detected",
                            runId, secondShotTargetName());
                    windmill.setPower(0);
                    shooter.setVelocity(0);
                    shootingState = "SECOND SHOT SKIPPED - DRIVING TO PARK";
                    if (!parkTwoFeetRobotLeft()) return;
                    shootingState = "PARKED - SECOND SHOT SKIPPED";
                    return;
                }
                if (!waitForShooterSpinup()) {
                    shootingState = "SHOOTER NOT READY - DRIVING TO PARK";
                    if (!parkTwoFeetRobotLeft()) return;
                    shootingState = "PARKED - SHOOTER NOT READY";
                    return;
                }
                runShooterAndWindmill(true, 4.0);

                stopIntake();
                shooter.setVelocity(0);
                windmill.setPower(0);
                shootingState = "SECOND SHOT COMPLETE - BACKING UP 10 INCHES";
                if (!driveRobotRelative(-10.0, 0.0,
                        SECOND_INTAKE_HEADING_DEGREES,
                        "Back up 10 inches after second shot",
                        true, 3.0)) return;
                beginLoweringArmForIntake();
                finishLoweringArmForIntake();
                startIntake();
                startShooter(secondShooterVelocity);
                shootingState = "SECOND SHOT COMPLETE - INTAKE FORWARD 10, LEFT 1";
                if (!driveRobotRelative(10.0, 1.0,
                        SECOND_INTAKE_HEADING_DEGREES,
                        "Intake forward 10 inches and left 1 inch",
                        true, 3.0)) return;

                shootingState = "FINAL BALLS - SETTLING IN INTAKE";
                sleep(500);
                if (!driveIntakeNudge(-3.0, 0.60)) return;
                if (!driveIntakeNudge(3.0, 0.60)) return;

                shootingState = "FINAL BALLS - SHOOTING";
                startIntake();
                shooter.setVelocity(secondShooterVelocity);
                runShooterAndWindmill(false, SHOOTING_SECONDS);
                stopIntake();
                shooter.setVelocity(0);
                windmill.setPower(0);
                shootingState = "FINAL SHOT COMPLETE - PARKING";
                if (!parkTwoFeetRobotLeft()) return;
                shootingState = "PARKED - FINAL SHOT COMPLETE";
            }
        } catch (ForcedParkException ignored) {
            forcedParkInProgress = true;
            stopDrive();
            stopIntake();
            shooter.setVelocity(0);
            windmill.setPower(0);
            shootingState = "5 SECONDS REMAINING - FORCED PARK";
            RobotLog.ww(TAG, "FORCED_PARK id=%s elapsed=%.1f", runId,
                    autonomousTimer.seconds());
            telemetry.addLine("5 seconds remain; stopping the sequence and parking now.");
            telemetry.addData("Autonomous elapsed", "%.1f sec",
                    autonomousTimer.seconds());
            addShootingTelemetry();
            telemetry.update();
            parkTwoFeetRobotLeft();
            shootingState = "PARKED - TIME LIMIT";
        } finally {
            RobotLog.ii(TAG, "RUN_END id=%s last_pose=(%.2f,%.2f,%.1f) state=%s",
                    runId, x, y, Math.toDegrees(heading), shootingState);
            stopDrive();
            stopIntake();
            shooter.setVelocity(0);
            windmill.setPower(0);
            if (limelight != null) limelight.stop();
            if (webcamPortal != null) webcamPortal.close();
            shootingState = "STOPPED";
        }

        telemetry.addLine("Complete");
        telemetry.addData("Final X / Y", "%.1f / %.1f in", x, y);
        addShootingTelemetry();
        telemetry.update();
        sleep(1000);
    }

    private boolean followFastSecondShootingPath() {
        if (!driveToPose(FAST_SECOND_PATH_WAYPOINT_1_X,
                FAST_SECOND_PATH_WAYPOINT_1_Y,
                FAST_SECOND_PATH_WAYPOINT_1_HEADING,
                "Fast shooting curve 1/2",
                18.0, Math.toRadians(35.0), false)) return false;
        if (!driveToPose(FAST_SECOND_PATH_WAYPOINT_2_X,
                FAST_SECOND_PATH_WAYPOINT_2_Y,
                FAST_SECOND_PATH_WAYPOINT_2_HEADING,
                "Fast shooting curve 2/2",
                18.0, Math.toRadians(35.0), false)) return false;
        if (!driveToPose(SECOND_INTAKE_X_INCHES,
                SECOND_INTAKE_Y_INCHES,
                SECOND_INTAKE_HEADING_DEGREES,
                "Second intake position",
                POSITION_TOLERANCE_INCHES, Math.toRadians(3.0), true,
                SECOND_SHOOTING_POSE_TIMEOUT_SECONDS,
                SECOND_SHOOTING_HEADING_TIMEOUT_SECONDS)) return false;
        return driveToPose(SECOND_SHOOTING_X_INCHES,
                SECOND_SHOOTING_Y_INCHES,
                SECOND_SHOOTING_CAPTURED_HEADING_DEGREES,
                "Fast curve final shooting position",
                POSITION_TOLERANCE_INCHES, Math.toRadians(3.0), true,
                SECOND_SHOOTING_POSE_TIMEOUT_SECONDS,
                SECOND_SHOOTING_HEADING_TIMEOUT_SECONDS);
    }

    protected String secondShotTargetName() {
        return "RED SCORING";
    }

    protected boolean isSecondShotTargetSingleTag(int id) {
        return id >= 30 && id <= 33;
    }

    private boolean waitForSecondShotTarget() {
        stopDrive();
        shootingState = "WAITING FOR " + secondShotTargetName() + " APRILTAG";
        ElapsedTime timer = new ElapsedTime();
        while (autonomousMayContinue() && timer.seconds() < SECOND_SHOT_TAG_TIMEOUT_SECONDS) {
            shooter.setVelocity(shooterTargetVelocity);
            String detected = "NONE";
            for (AprilTagDetection detection : aprilTags.getDetections()) {
                boolean target = false;
                if (detection instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                    detected = cluster.metadata == null ? "unknown cluster"
                            : cluster.metadata.name;
                    target = detected.equalsIgnoreCase(secondShotTargetName());
                } else if (detection instanceof AprilTagSingleDetection) {
                    int id = ((AprilTagSingleDetection) detection).id;
                    detected = "tag " + id;
                    target = isSecondShotTargetSingleTag(id);
                }
                if (target) {
                    RobotLog.ii(TAG, "SECOND_SHOT_TAG_FOUND id=%s target=%s detected=%s elapsed=%.2f",
                            runId, secondShotTargetName(), detected, timer.seconds());
                    shootingState = secondShotTargetName() + " APRILTAG FOUND";
                    return true;
                }
            }
            telemetry.addData("Step", "Recognizing %s AprilTag", secondShotTargetName());
            telemetry.addData("Detected", detected);
            telemetry.addData("Tag timeout", "%.1f / %.1f sec",
                    timer.seconds(), SECOND_SHOT_TAG_TIMEOUT_SECONDS);
            telemetry.addData("Webcam", webcamPortal.getCameraState());
            addShootingTelemetry();
            telemetry.update();
            idle();
        }
        return false;
    }

    private boolean parkTwoFeetRobotLeft() {
        updatePose();
        RobotLog.ii(TAG,
                "RELATIVE_PARK_LEFT id=%s start=(%.2f,%.2f,%.1f) target=(%.2f,%.2f,%.1f)",
                runId, x, y, Math.toDegrees(heading),
                FINAL_PARK_X_INCHES, FINAL_PARK_Y_INCHES,
                FINAL_PARK_HEADING_DEGREES);
        if (!followLineSegment(FINAL_PARK_X_INCHES, FINAL_PARK_Y_INCHES,
                FINAL_PARK_HEADING_DEGREES,
                "Segment: park at recorded field position", 7.0)) return false;
        return driveRobotRelative(-5.0, 0.0,
                FINAL_PARK_HEADING_DEGREES,
                "Back up 5 inches after reaching park",
                true, 2.0);
    }

    private void startShooter(double targetVelocity) {
        shootingState = "SHOOTER SPIN-UP";
        shooterTargetVelocity = targetVelocity;
        activeShooterReadyVelocity =
                SHOOTER_READY_FRACTION_OF_TARGET * targetVelocity;
        shooter.setVelocity(shooterTargetVelocity);
        RobotLog.ii(TAG, "Shooter spin-up started");
    }

    private boolean autonomousMayContinue() {
        if (!forcedParkInProgress
                && autonomousTimer.seconds() >= FORCED_PARK_TIME_SECONDS) {
            throw new ForcedParkException();
        }
        return opModeIsActive();
    }

    private static final class ForcedParkException extends RuntimeException {
    }

    private void keepShooterOffBeforeSecondIntake() {
        shooter.setVelocity(0);
        windmill.setPower(0);
    }

    private boolean waitForShooterSpinup() {
        ElapsedTime spinupTimer = new ElapsedTime();
        while (autonomousMayContinue()
                && spinupTimer.seconds() < SHOOTER_SPINUP_TIMEOUT_SECONDS) {
            shooter.setVelocity(shooterTargetVelocity);
            double velocity = Math.abs(shooter.getVelocity());
            if (velocity >= activeShooterReadyVelocity) {
                RobotLog.ii(TAG, "Shooter ready at %.0f ticks/s after %.1f seconds",
                        velocity, spinupTimer.seconds());
                return true;
            }
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
        shootingState = "SHOOTER FAILED TO REACH SPEED";
        RobotLog.ee(TAG, "Shooter only reached %.0f of %.0f ticks/s",
                velocity, activeShooterReadyVelocity);
        shooter.setVelocity(0);
        return false;
    }

    private void runShooterAndWindmill() {
        runShooterAndWindmill(false);
    }

    private void runShooterAndWindmill(boolean forceWindmillFeed) {
        runShooterAndWindmill(forceWindmillFeed, SHOOTING_SECONDS);
    }

    private void runShooterAndWindmill(boolean forceWindmillFeed,
                                       double shootingSeconds) {
        shootingState = "SHOOTER + WINDMILL RUNNING";
        ElapsedTime shootingTimer = new ElapsedTime();
        RobotLog.ii(TAG, forceWindmillFeed
                        ? "Windmill forced to full feed for second four-ball shot"
                        : "Windmill feeds at full power only above %.0f ticks/s",
                activeShooterReadyVelocity);
        while (autonomousMayContinue() && shootingTimer.seconds() < shootingSeconds) {
            // Hold one velocity for every ball; do not reduce the target late
            // in the cycle while the windmill is still feeding.
            shooter.setVelocity(shooterTargetVelocity);
            double actualVelocity = Math.abs(shooter.getVelocity());
            boolean readyToFeed = actualVelocity >= activeShooterReadyVelocity;
            // -1.0 is the CR servo's maximum available software speed.
            boolean feeding = forceWindmillFeed || readyToFeed;
            windmill.setPower(feeding ? WINDMILL_FEED_POWER : 0.0);
            telemetry.addData("Step", "Continuous shooting");
            telemetry.addData("Shooting time", "%.1f / %.1f sec",
                    shootingTimer.seconds(), shootingSeconds);
            telemetry.addData("Windmill", feeding
                    ? "FULL SPEED" : "WAITING FOR SHOOTER SPEED");
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

        ElapsedTime intakeLeadTimer = new ElapsedTime();
        while (autonomousMayContinue()
                && intakeLeadTimer.seconds() < SECOND_POSITION_INTAKE_LEAD_SECONDS) {
            // Hold a straight, repeatable press so the left intake wheel's
            // contact point can be calibrated against the lower pollen.
            holdStraightIntakePress(pressCenterTarget, pressHeadingTarget,
                    SECOND_POSITION_CONTACT_FORWARD_POWER);
            shooter.setVelocity(shooterVelocity);
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
        windmill.setPower(WINDMILL_FEED_POWER);
        ElapsedTime timer = new ElapsedTime();

        while (autonomousMayContinue() && timer.seconds() < SECOND_SHOOTING_SECONDS) {
            // Do not run odometry pose correction here: the powered intake
            // shares the left odometry encoder channel. Keep a simple forward
            // press while intake, shooter, and windmill feed together.
            holdStraightIntakePress(pressCenterTarget, pressHeadingTarget,
                    SECOND_POSITION_HOLD_FORWARD_POWER);
            shooterTargetVelocity = shooterVelocity;
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
        stopIntake();
        shooter.setVelocity(0);
        windmill.setPower(0);
        shootingState = "PUSH TO RED INTAKE POSITION";

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        syncPodBaselines();

        ElapsedTime pathTimer = new ElapsedTime();
        ElapsedTime pathLogTimer = new ElapsedTime();
        RobotLog.ii(TAG, "RED INTAKE PATH START id=%s pose=(%.2f,%.2f,%.1f)",
                runId, x, y, Math.toDegrees(heading));

        while (autonomousMayContinue() && !gamepad1.b) {
            updatePose();
            stopDrive();
            if (pathLogTimer.milliseconds() >= 100) {
                RobotLog.ii(TAG,
                        "RED INTAKE PATH id=%s time=%.2f x=%.2f y=%.2f heading=%.1f",
                        runId, pathTimer.seconds(), x, y, Math.toDegrees(heading));
                pathLogTimer.reset();
            }
            telemetry.addData("Mode", "PUSH ROBOT TO RED INTAKE POSITION");
            telemetry.addData("X forward", "%+.2f in", x);
            telemetry.addData("Y left", "%+.2f in", y);
            telemetry.addData("Heading CCW", "%+.1f deg", Math.toDegrees(heading));
            telemetry.addLine("Press B to capture this pose");
            telemetry.addLine("No timeout - waits until B or Stop");
            telemetry.update();
            idle();
        }
        if (!opModeIsActive()) return;

        updatePose();
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopDrive();
        RobotLog.ii(TAG,
                "RED INTAKE POSE id=%s time=%.2f x=%.2f y=%.2f heading=%.1f",
                runId, pathTimer.seconds(), x, y, Math.toDegrees(heading));
        while (autonomousMayContinue()) {
            stopDrive();
            telemetry.addData("Mode", "RED INTAKE POSE CAPTURED");
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
        while (autonomousMayContinue() && gamepad1.a) idle();

        while (autonomousMayContinue() && !gamepad1.b) {
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

        while (autonomousMayContinue()) {
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

        while (autonomousMayContinue()
                && timer.seconds() < INTAKE_FORWARD_TIMEOUT_SECONDS) {
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
        while (autonomousMayContinue()
                && intakeTimer.seconds() < INTAKE_AFTER_DRIVE_SECONDS) {
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

    /** Overridden only by the calibration OpMode that records the next waypoint. */
    protected boolean captureNextPositionAfterIntake() {
        return false;
    }

    private void stopIntake() {
        intakeMotor.setPower(0);
    }

    private void startIntake() {
        intakeMotor.setPower(1.0);
    }

    private void beginLoweringArmForIntake() {
        flowerArm.setPosition(ARM_DOWN_POSITION);
        RobotLog.ii(TAG, "ARM_LOWERED id=%s position=%.2f", runId, ARM_DOWN_POSITION);
    }

    private void finishLoweringArmForIntake() {
        sleep(ARM_LOWER_SETTLE_MS);
    }

    private void raiseArmForParking() {
        flowerArm.setPosition(ARM_UP_POSITION);
        RobotLog.ii(TAG, "ARM_RAISED id=%s position=%.2f", runId, ARM_UP_POSITION);
        sleep(ARM_RAISE_SETTLE_MS);
    }

    private boolean driveIntakeNudge(double distanceInches) {
        return driveIntakeNudge(distanceInches, INTAKE_NUDGE_POWER);
    }

    private boolean driveIntakeNudge(double distanceInches, double nudgePower) {
        int startingRightTicks = readRight();
        double direction = Math.signum(distanceInches);
        double targetDistance = Math.abs(distanceInches);
        double timeoutSeconds = Math.max(INTAKE_NUDGE_TIMEOUT_SECONDS,
                targetDistance / 3.0);
        ElapsedTime timer = new ElapsedTime();
        while (autonomousMayContinue() && timer.seconds() < timeoutSeconds) {
            startIntake();
            double signedDistance =
                    (readRight() - startingRightTicks) / RIGHT_TICKS_PER_INCH;
            if (signedDistance * direction
                    >= targetDistance - INTAKE_FORWARD_TOLERANCE_INCHES) break;
            setMecanum(direction * nudgePower, 0, 0);
            telemetry.addData("Step", "Intake nudge %s %.1f inches",
                    direction > 0 ? "forward" : "backward", targetDistance);
            telemetry.update();
            idle();
        }
        stopDrive();
        updatePose();
        return opModeIsActive();
    }

    private boolean recordSecondShootingPathAfterIntake() {
        stopDrive();
        stopIntake();
        shooter.setVelocity(0);
        windmill.setPower(0);
        shootingState = "PUSH TO SECOND SHOOTING POSITION";

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        syncPodBaselines();

        ElapsedTime pathTimer = new ElapsedTime();
        ElapsedTime pathLogTimer = new ElapsedTime();
        RobotLog.ii(TAG, "SECOND_SHOOTING_PATH_START id=%s pose=(%.2f,%.2f,%.1f)",
                runId, x, y, Math.toDegrees(heading));

        while (autonomousMayContinue() && !gamepad1.b) {
            updatePose();
            stopDrive();
            if (pathLogTimer.milliseconds() >= 100) {
                RobotLog.ii(TAG,
                        "SECOND_SHOOTING_PATH id=%s time=%.2f x=%.2f y=%.2f heading=%.1f",
                        runId, pathTimer.seconds(), x, y, Math.toDegrees(heading));
                pathLogTimer.reset();
            }
            telemetry.addData("Mode", "PUSH TO SECOND SHOOTING POSITION");
            telemetry.addData("X forward", "%+.2f in", x);
            telemetry.addData("Y left", "%+.2f in", y);
            telemetry.addData("Heading CCW", "%+.1f deg", Math.toDegrees(heading));
            telemetry.addLine("Press B to capture the final pose");
            telemetry.addLine("No timeout - waits until B or Stop");
            telemetry.update();
            idle();
        }
        if (!opModeIsActive()) return false;

        updatePose();
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopDrive();
        RobotLog.ii(TAG,
                "SECOND_SHOOTING_POSE id=%s time=%.2f x=%.2f y=%.2f heading=%.1f",
                runId, pathTimer.seconds(), x, y, Math.toDegrees(heading));

        while (autonomousMayContinue()) {
            stopDrive();
            telemetry.addData("Mode", "SECOND SHOOTING POSE CAPTURED");
            telemetry.addData("RECORDED X forward", "%+.2f in", x);
            telemetry.addData("RECORDED Y left", "%+.2f in", y);
            telemetry.addData("RECORDED heading CCW", "%+.1f deg",
                    Math.toDegrees(heading));
            telemetry.addLine("Write down these values, then press Stop");
            telemetry.update();
            idle();
        }
        return false;
    }

    private boolean alignToPollenColumn() {
        if (limelight == null) {
            RobotLog.ii(TAG, "VISION_ALIGN_SKIPPED id=%s Limelight disconnected", runId);
            telemetry.addLine("Limelight disconnected; skipping pollen alignment.");
            telemetry.update();
            return true;
        }
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

        while (autonomousMayContinue() && timer.seconds() < POLLEN_ALIGNMENT_TIMEOUT_SECONDS) {
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
        return driveToPose(targetX, targetY, targetHeadingDegrees, step,
                positionTolerance, headingTolerance, stopAtTarget,
                timeoutSeconds, Double.POSITIVE_INFINITY);
    }

    private boolean driveRobotRelative(double forwardInches, double leftInches,
                                       double headingDegrees, String step,
                                       boolean stopAtTarget, double timeoutSeconds) {
        updatePose();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double targetX = x + forwardInches * cos - leftInches * sin;
        double targetY = y + forwardInches * sin + leftInches * cos;
        return driveToPose(targetX, targetY, headingDegrees, step,
                0.75, Math.toRadians(4.0), stopAtTarget, timeoutSeconds);
    }

    private boolean turnInPlaceToHeading(double targetHeadingDegrees,
                                         String step, double timeoutSeconds) {
        return turnCounterClockwiseBy(180.0, step, timeoutSeconds);
    }

    private boolean turnCounterClockwiseBy(double rotationDegrees,
                                           String step, double timeoutSeconds) {
        final double tolerance = Math.toRadians(3.0);
        final double turnKp = 1.25;
        final double maxPower = 0.90;
        final double minPower = 0.22;
        ElapsedTime timer = new ElapsedTime();
        updatePose();
        double previousHeading = heading;
        double counterClockwiseRotation = 0.0;
        final double requestedRotation = Math.toRadians(rotationDegrees);

        while (autonomousMayContinue() && timer.seconds() < timeoutSeconds) {
            updatePose();
            // Accumulate the measured CCW rotation instead of calculating the
            // shortest path to -180. At exactly 180 degrees, tiny noise around
            // zero otherwise flips the shortest path between CW and CCW.
            // Direction is fixed by the motor command below. Accumulate the
            // magnitude so an encoder-sign mismatch cannot prevent stopping.
            counterClockwiseRotation += Math.abs(
                    normalizeRadians(heading - previousHeading));
            previousHeading = heading;
            double remaining = requestedRotation - counterClockwiseRotation;
            if (remaining <= tolerance) {
                stopDrive();
                RobotLog.ii(TAG, "TURN_REACHED id=%s step=%s elapsed=%.2f heading=%.1f",
                        runId, step, timer.seconds(), Math.toDegrees(heading));
                return true;
            }

            // Keep one fixed physical direction for the entire half-turn.
            double turn = Range.clip(remaining * turnKp, minPower, maxPower);
            setMecanum(0.0, 0.0, -turn);
            telemetry.addData("Step", step);
            telemetry.addData("CCW rotation", "%.1f / %.1f deg",
                    Math.toDegrees(counterClockwiseRotation), rotationDegrees);
            telemetry.addData("Turn power", "%+.2f", -turn);
            telemetry.update();
            idle();
        }

        stopDrive();
        RobotLog.ww(TAG, "TURN_TIMEOUT id=%s step=%s heading=%.1f rotation=%.1f",
                runId, step, Math.toDegrees(heading), rotationDegrees);
        return false;
    }

    private boolean driveStraightForward(double distanceInches,
                                         String step, double timeoutSeconds) {
        updatePose();
        final double targetHeading = heading;
        final double forwardPower = 0.75;
        final double headingKp = 1.20;
        final double maxHeadingCorrection = 0.45;
        int startLeft = readLeft();
        int startRight = readRight();
        ElapsedTime timer = new ElapsedTime();

        while (autonomousMayContinue() && timer.seconds() < timeoutSeconds) {
            updatePose();
            double leftDistance = Math.abs(readLeft() - startLeft)
                    / LEFT_TICKS_PER_INCH;
            double rightDistance = Math.abs(readRight() - startRight)
                    / RIGHT_TICKS_PER_INCH;
            double traveled = (leftDistance + rightDistance) / 2.0;
            if (traveled >= distanceInches) {
                stopDrive();
                RobotLog.ii(TAG,
                        "STRAIGHT_REACHED id=%s step=%s elapsed=%.2f distance=%.1f pose=(%.2f,%.2f,%.1f)",
                        runId, step, timer.seconds(), traveled,
                        x, y, Math.toDegrees(heading));
                return true;
            }

            // Isolate the feedback loops: hold only the heading here. X/Y and
            // lateral corrections stay disabled so they cannot fight forward
            // travel. Reduced forward power preserves turning authority.
            double headingError = normalizeRadians(targetHeading - heading);
            double headingCorrection = Range.clip(
                    headingError * headingKp,
                    -maxHeadingCorrection, maxHeadingCorrection);
            // Negative mixer turn produces positive/CCW odometry heading.
            setMecanum(forwardPower, 0.0, -headingCorrection);
            telemetry.addData("Step", step);
            telemetry.addData("Straight distance", "%.1f / %.1f in",
                    traveled, distanceInches);
            telemetry.addData("Heading hold", "%+.1f / %+.1f deg",
                    Math.toDegrees(heading), Math.toDegrees(targetHeading));
            telemetry.addData("Heading correction", "%+.2f", -headingCorrection);
            telemetry.update();
            idle();
        }

        stopDrive();
        RobotLog.ww(TAG, "STRAIGHT_TIMEOUT id=%s step=%s pose=(%.2f,%.2f,%.1f)",
                runId, step, x, y, Math.toDegrees(heading));
        return false;
    }

    private boolean followLineSegment(double targetX, double targetY,
                                      double targetHeadingDegrees,
                                      String step, double timeoutSeconds) {
        return followLineSegment(targetX, targetY, targetHeadingDegrees,
                step, timeoutSeconds, Double.NaN);
    }

    private boolean followLineSegment(double targetX, double targetY,
                                      double targetHeadingDegrees,
                                      String step, double timeoutSeconds,
                                      double armRaiseProgressFraction) {
        updatePose();
        final double startX = x;
        final double startY = y;
        final double segmentX = targetX - startX;
        final double segmentY = targetY - startY;
        final double segmentLength = Math.hypot(segmentX, segmentY);
        if (segmentLength < 0.01) return true;

        final double unitX = segmentX / segmentLength;
        final double unitY = segmentY / segmentLength;
        final double targetHeading = Math.toRadians(targetHeadingDegrees);
        final double lookaheadInches = 10.0;
        final double maxTranslation = 0.95;
        final double minTranslation = 0.20;
        final double rampDistance = 14.0;
        final double headingKp = 1.15;
        final double maxTurn = 0.50;
        ElapsedTime timer = new ElapsedTime();
        boolean armRaised = false;

        while (autonomousMayContinue() && timer.seconds() < timeoutSeconds) {
            updatePose();
            double fromStartX = x - startX;
            double fromStartY = y - startY;
            double progress = fromStartX * unitX + fromStartY * unitY;
            double crossTrack = unitX * fromStartY - unitY * fromStartX;
            double remaining = Math.hypot(targetX - x, targetY - y);
            double headingError = normalizeRadians(targetHeading - heading);

            if (!armRaised && !Double.isNaN(armRaiseProgressFraction)
                    && progress >= segmentLength * armRaiseProgressFraction) {
                flowerArm.setPosition(ARM_UP_POSITION);
                armRaised = true;
                RobotLog.ii(TAG,
                        "ARM_RAISED_WHILE_DRIVING id=%s position=%.2f progress=%.1f/%.1f",
                        runId, ARM_UP_POSITION, progress, segmentLength);
            }

            if (remaining <= 2.0 && Math.abs(headingError) <= Math.toRadians(4.0)) {
                stopDrive();
                RobotLog.ii(TAG,
                        "SEGMENT_REACHED id=%s step=%s elapsed=%.2f remaining=%.2f cross=%.2f pose=(%.2f,%.2f,%.1f)",
                        runId, step, timer.seconds(), remaining, crossTrack,
                        x, y, Math.toDegrees(heading));
                return true;
            }

            double lookaheadProgress = Range.clip(
                    progress + lookaheadInches, 0.0, segmentLength);
            double aimX = startX + unitX * lookaheadProgress;
            double aimY = startY + unitY * lookaheadProgress;
            if (segmentLength - progress <= lookaheadInches) {
                aimX = targetX;
                aimY = targetY;
            }
            double aimErrorX = aimX - x;
            double aimErrorY = aimY - y;
            double aimLength = Math.max(0.001, Math.hypot(aimErrorX, aimErrorY));

            double rampUp = Range.clip((progress + 3.0) / rampDistance, 0.35, 1.0);
            double rampDown = Range.clip(remaining / rampDistance, 0.0, 1.0);
            double translation = Range.clip(maxTranslation * Math.min(rampUp, rampDown),
                    minTranslation, maxTranslation);
            // Prioritize completing large heading changes before translating.
            double headingScale = Range.clip(
                    1.0 - Math.abs(headingError) / Math.toRadians(100.0),
                    0.25, 1.0);
            translation *= headingScale;

            double fieldForwardX = translation * aimErrorX / aimLength;
            double fieldForwardY = translation * aimErrorY / aimLength;
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double robotForward = fieldForwardX * cos + fieldForwardY * sin;
            double robotLeft = -fieldForwardX * sin + fieldForwardY * cos;
            double headingCorrection = Range.clip(
                    headingError * headingKp, -maxTurn, maxTurn);
            double mixerTurn = -headingCorrection;

            // Reserve wheel range for turn instead of allowing normalization
            // to weaken heading control at high translation power.
            double availableForTranslation = Math.max(0.20, 1.0 - Math.abs(mixerTurn));
            double translationWheelMax = Math.max(
                    Math.abs(robotForward + robotLeft),
                    Math.abs(robotForward - robotLeft));
            if (translationWheelMax > availableForTranslation) {
                double scale = availableForTranslation / translationWheelMax;
                robotForward *= scale;
                robotLeft *= scale;
            }

            setMecanum(robotForward, -robotLeft, mixerTurn);
            telemetry.addData("Step", step);
            telemetry.addData("Segment progress", "%.1f / %.1f in",
                    progress, segmentLength);
            telemetry.addData("Cross-track / remaining", "%+.1f / %.1f in",
                    crossTrack, remaining);
            telemetry.addData("Heading", "%+.1f / %+.1f deg",
                    Math.toDegrees(heading), targetHeadingDegrees);
            telemetry.addData("Command F / L / T", "%+.2f / %+.2f / %+.2f",
                    robotForward, robotLeft, mixerTurn);
            telemetry.update();
            idle();
        }

        stopDrive();
        RobotLog.ww(TAG,
                "SEGMENT_TIMEOUT id=%s step=%s pose=(%.2f,%.2f,%.1f) target=(%.2f,%.2f,%.1f)",
                runId, step, x, y, Math.toDegrees(heading),
                targetX, targetY, targetHeadingDegrees);
        return false;
    }

    private boolean driveFixedVectorToPose(double targetX, double targetY,
                                           double targetHeadingDegrees,
                                           String step, double timeoutSeconds) {
        updatePose();
        final double startX = x;
        final double startY = y;
        final double targetHeading = Math.toRadians(targetHeadingDegrees);
        final double errorX = targetX - x;
        final double errorY = targetY - y;
        final double commandedDistance = Math.hypot(errorX, errorY);
        final double cos = Math.cos(heading);
        final double sin = Math.sin(heading);
        final double robotForward = errorX * cos + errorY * sin;
        final double robotLeft = -errorX * sin + errorY * cos;
        final double vectorLength = Math.max(0.001,
                Math.hypot(robotForward, robotLeft));
        final double translationPower = 0.80;
        final double forwardCommand = translationPower * robotForward / vectorLength;
        final double leftCommand = translationPower * robotLeft / vectorLength;
        ElapsedTime timer = new ElapsedTime();

        while (autonomousMayContinue() && timer.seconds() < timeoutSeconds) {
            updatePose();
            double remaining = Math.hypot(targetX - x, targetY - y);
            double traveled = Math.hypot(x - startX, y - startY);
            if (remaining <= 2.5 || traveled >= commandedDistance) {
                stopDrive();
                RobotLog.ii(TAG,
                        "FIXED_VECTOR_REACHED id=%s step=%s elapsed=%.2f remaining=%.1f pose=(%.2f,%.2f,%.1f)",
                        runId, step, timer.seconds(), remaining,
                        x, y, Math.toDegrees(heading));
                return true;
            }

            double headingError = normalizeRadians(targetHeading - heading);
            double headingCorrection = Range.clip(
                    headingError * 1.20, -0.40, 0.40);
            // Keep the translation vector fixed. Only heading is corrected.
            setMecanum(forwardCommand, -leftCommand, -headingCorrection);
            telemetry.addData("Step", step);
            telemetry.addData("Remaining", "%.1f in", remaining);
            telemetry.addData("Fixed drive F / L", "%+.2f / %+.2f",
                    forwardCommand, leftCommand);
            telemetry.addData("Heading", "%+.1f / %+.1f deg",
                    Math.toDegrees(heading), targetHeadingDegrees);
            telemetry.update();
            idle();
        }

        stopDrive();
        RobotLog.ww(TAG, "FIXED_VECTOR_TIMEOUT id=%s step=%s pose=(%.2f,%.2f,%.1f)",
                runId, step, x, y, Math.toDegrees(heading));
        return false;
    }

    private boolean driveToPose(double targetX, double targetY,
                                double targetHeadingDegrees, String step,
                                double positionTolerance, double headingTolerance,
                                boolean stopAtTarget, double timeoutSeconds,
                                double headingTimeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime headingTimer = new ElapsedTime();
        boolean headingTimerStarted = false;
        double targetHeading = Math.toRadians(targetHeadingDegrees);
        RobotLog.ii(TAG, "WAYPOINT_START id=%s step=%s target=(%.2f,%.2f,%.1f) "
                        + "start=(%.2f,%.2f,%.1f) timeout=%.1f",
                runId, step, targetX, targetY, targetHeadingDegrees,
                x, y, Math.toDegrees(heading), timeoutSeconds);

        while (autonomousMayContinue() && timer.seconds() < timeoutSeconds) {
            // Intake runs along the collection path, but stops after the
            // timed half-second hold before traveling to the next position.
            if (runIntakeDuringPath) startIntake();
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

            if (positionError <= positionTolerance && !headingTimerStarted) {
                headingTimer.reset();
                headingTimerStarted = true;
                RobotLog.ii(TAG, "HEADING_SETTLE_START id=%s step=%s timeout=%.1f",
                        runId, step, headingTimeoutSeconds);
            }
            if (headingTimerStarted
                    && headingTimer.seconds() >= headingTimeoutSeconds) {
                stopDrive();
                RobotLog.ww(TAG, "HEADING_SETTLE_TIMEOUT id=%s step=%s elapsed=%.2f "
                                + "pose=(%.2f,%.2f,%.1f) heading_error=%.1f; continuing",
                        runId, step, headingTimer.seconds(), x, y,
                        Math.toDegrees(heading), Math.toDegrees(headingError));
                telemetry.addData("Status",
                        "Heading timed out after %.1f sec; continuing",
                        headingTimeoutSeconds);
                telemetry.update();
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

            // Scale both translation axes together. Clipping them separately
            // turns a shallow diagonal into equal full forward/strafe power,
            // which powers only two wheels and makes the controller hunt.
            double rawForward = robotForwardError * TRANSLATION_KP;
            double rawLeft = robotLeftError * TRANSLATION_KP;
            double translationScale = Math.max(1.0, Math.max(
                    Math.abs(rawForward) / MAX_FORWARD_POWER,
                    Math.abs(rawLeft) / MAX_STRAFE_POWER));
            double forward = minimumPower(rawForward / translationScale,
                    MIN_FORWARD_POWER, robotForwardError);
            double left = minimumPower(rawLeft / translationScale,
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
            if (headingTimerStarted && !Double.isInfinite(headingTimeoutSeconds)) {
                telemetry.addData("Heading settle time", "%.1f / %.1f sec",
                        headingTimer.seconds(), headingTimeoutSeconds);
            }
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
        if (limelight == null) return;
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

        // With the installed pod directions, left-minus-right is positive for
        // a counterclockwise turn.
        double deltaHeading = (deltaLeft - deltaRight) / POD_SPACING_INCHES;
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
