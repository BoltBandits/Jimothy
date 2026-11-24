package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

/**
 * Red-side upgraded TeleOp with a continuously spinning, distance-adjusted shooter.
 * Right bumper remains the driver's command to feed balls with the windmill.
 */
@TeleOp(name = "Red Teleop", group = "Jimothy")
public class RedTeleOp extends JimothyUpgradedTeleOp {
    private static final String TAG = "RedTeleOpCalibration";
    // Calibrated wheel-RPM anchors. Closed-loop velocity control holds these
    // speeds as battery voltage changes.
    private static final double CLOSE_DISTANCE_INCHES = 30.0;
    private static final double MID_DISTANCE_INCHES = 34.0;
    private static final double LONG_DISTANCE_INCHES = 40.0;
    private static final double FAR_DISTANCE_INCHES = 120.0;
    private static final double CLOSE_SHOOTER_RPM = 29.0;
    private static final double MID_SHOOTER_RPM = 31.5;
    private static final double LONG_SHOOTER_RPM = 33.0;
    private static final double FAR_SHOOTER_RPM = 34.2;

    // Camera mounting offsets measured with the red scoring target physically
    // upright and the robot aimed correctly at about 30 inches.
    // Successful calibration groups place the target slightly to the camera's
    // right. Interpolate the desired bearing and tolerance by approach angle.
    private static final double NEGATIVE_45_TARGET_BEARING_DEGREES = 2.8;
    private static final double ZERO_TARGET_BEARING_DEGREES = 4.0;
    private static final double POSITIVE_45_TARGET_BEARING_DEGREES = 2.8;
    // The scoring opening appears wider near a head-on approach. Successful
    // front-position shots near -15 degrees support a roughly symmetrical
    // window around the calibrated preferred bearing.
    private static final double NEGATIVE_45_LEFT_TOLERANCE_DEGREES = 4.0;
    private static final double ZERO_LEFT_TOLERANCE_DEGREES = 22.0;
    private static final double POSITIVE_45_LEFT_TOLERANCE_DEGREES = 4.0;
    private static final double NEGATIVE_45_RIGHT_TOLERANCE_DEGREES = 5.0;
    private static final double ZERO_RIGHT_TOLERANCE_DEGREES = 22.0;
    private static final double POSITIVE_45_RIGHT_TOLERANCE_DEGREES = 5.0;
    // Closer than this, the fixed launcher elevation is too flat to clear the
    // lower rim reliably even when wheel speed and horizontal aim are correct.
    private static final double NEGATIVE_45_MIN_DISTANCE_INCHES = 27.0;
    private static final double ZERO_MIN_DISTANCE_INCHES = 24.0;
    private static final double POSITIVE_45_MIN_DISTANCE_INCHES = 27.0;
    private static final double NEAR_MAX_YAW_ERROR_DEGREES = 50.0;
    private static final double FAR_MAX_YAW_ERROR_DEGREES = 20.0;
    // At an oblique approach only one of the four scoring tags may be visible.
    // The cluster identity still guarantees it belongs to the red target.
    private static final int MIN_CLUSTER_VISIBILITY_PERCENT = 25;
    private static final double READY_SPEED_FRACTION = 0.95;
    private static final double TARGET_MEMORY_SECONDS = 3.0;
    private static final double GREEN_CONFIRM_SECONDS = 0.05;
    private static final double RED_CONFIRM_SECONDS = 0.10;
    private static final double AUTO_AIM_TOLERANCE_DEGREES = 1.5;
    private static final double AUTO_AIM_FAR_KP = 0.012;
    private static final double AUTO_AIM_MID_KP = 0.009;
    private static final double AUTO_AIM_NEAR_KP = 0.006;
    private static final double AUTO_AIM_MIN_POWER = 0.025;
    private static final double AUTO_AIM_MAX_POWER = 0.18;
    private static final double AUTO_AIM_FILTER_ALPHA = 0.85;
    private static final double LEFT_TICKS_PER_INCH = 503.47;
    private static final double RIGHT_TICKS_PER_INCH = 504.53;
    private static final double CENTER_TICKS_PER_INCH = 510.30;
    private static final double POD_SPACING_INCHES = 12.10;
    private static final double CENTER_POD_X_INCHES = -6.56;
    private static final int MAP_ANCHOR_FRAMES = 15;
    private static final double MAP_CORRECTION_ALPHA = 0.02;
    private static final double MAP_MAX_CORRECTION_ERROR_INCHES = 12.0;
    private static final double VIBRATION_LOG_PERIOD_SECONDS = 0.05;

    private TargetObservation target = TargetObservation.none();
    private TargetObservation lastValidTarget = TargetObservation.none();
    private long lastValidTargetNanos;
    private boolean lastReadyToShoot;
    private boolean pendingReadyToShoot;
    private boolean displayedReadyToShoot;
    private long pendingReadySinceNanos;
    private boolean lastFeedRequested;
    private DcMotor leftPod;
    private DcMotor rightPod;
    private DcMotor centerPod;
    private DcMotor diagnosticLeftFront;
    private DcMotor diagnosticRightFront;
    private DcMotor diagnosticLeftBack;
    private DcMotor diagnosticRightBack;
    private boolean odometryAvailable;
    private int previousLeftTicks;
    private int previousRightTicks;
    private int previousCenterTicks;
    private double odometryX;
    private double odometryY;
    private double odometryHeading;
    private boolean targetMapReady;
    private int mapAnchorFrames;
    private double mapAnchorTargetXSum;
    private double mapAnchorTargetYSum;
    private double mappedTargetX;
    private double mappedTargetY;
    private boolean lastMapResetButton;
    private double filteredAimErrorDegrees = Double.NaN;
    private final ElapsedTime vibrationLogTimer = new ElapsedTime();

    @Override
    public void init() {
        super.init();
        diagnosticLeftFront = hardwareMap.tryGet(DcMotor.class, RobotHookups.Names.FRONT_LEFT);
        diagnosticRightFront = hardwareMap.tryGet(DcMotor.class, RobotHookups.Names.FRONT_RIGHT);
        diagnosticLeftBack = hardwareMap.tryGet(DcMotor.class, RobotHookups.Names.BACK_LEFT);
        diagnosticRightBack = hardwareMap.tryGet(DcMotor.class, RobotHookups.Names.BACK_RIGHT);
        leftPod = hardwareMap.tryGet(DcMotor.class, RobotHookups.Names.FRONT_LEFT);
        rightPod = hardwareMap.tryGet(DcMotor.class, "odo_right");
        centerPod = hardwareMap.tryGet(DcMotor.class, "odo_center");
        odometryAvailable = leftPod != null && rightPod != null && centerPod != null;
        if (odometryAvailable) {
            rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            centerPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            resetOdometryAndMap();
            telemetry.addLine("Odometry mapper ready: start still at farthest 0-deg position");
        } else {
            telemetry.addLine("Odometry mapper disabled: missing odo_right or odo_center");
        }
    }

    @Override
    public void start() {
        resetOdometryAndMap();
    }

    @Override
    public void loop() {
        updateOdometry();
        super.loop();
        updateTargetMap();
        if (vibrationLogTimer.seconds() >= VIBRATION_LOG_PERIOD_SECONDS) {
            vibrationLogTimer.reset();
            RobotLog.ii(TAG,
                    "FLYWHEEL alliance=%s target=%.0f actual=%.0f power=%.3f visible=%d range=%s feed=%d drive=%.3f/%.3f/%.3f/%.3f sticks=%.3f/%.3f/%.3f aim=%d",
                    teleOpAllianceName(), launcherTargetVelocity,
                    launcher == null ? 0.0 : Math.abs(launcher.getVelocity()),
                    launcher == null ? 0.0 : launcher.getPower(),
                    target.visible ? 1 : 0,
                    target.hasRange ? String.format(java.util.Locale.US, "%.2f", target.rangeInches)
                            : "none",
                    gamepad2.right_bumper ? 1 : 0,
                    diagnosticLeftFront == null ? 0.0 : diagnosticLeftFront.getPower(),
                    diagnosticRightFront == null ? 0.0 : diagnosticRightFront.getPower(),
                    diagnosticLeftBack == null ? 0.0 : diagnosticLeftBack.getPower(),
                    diagnosticRightBack == null ? 0.0 : diagnosticRightBack.getPower(),
                    gamepad1.left_stick_y, gamepad1.left_stick_x,
                    gamepad1.right_stick_x, gamepad2.a ? 1 : 0);
        }
    }

    @Override
    protected void updateTurbo() {
        // Automatic distance control replaces the four manual speed presets in
        // this mode. With no usable tag, retain the current 45% default.
        turbo = false;
        launcherTargetVelocity = NORMAL_SHOOTER_FRACTION * launcherMaxVelocity;
    }

    @Override
    protected void updateAutomaticShooterTarget() {
        target = findTargetWithMemory();
        if (target.hasRange) {
            double targetRpm;
            if (target.rangeInches <= MID_DISTANCE_INCHES) {
                double ratio = Range.clip(
                        (target.rangeInches - CLOSE_DISTANCE_INCHES)
                                / (MID_DISTANCE_INCHES - CLOSE_DISTANCE_INCHES),
                        0.0, 1.0);
                targetRpm = CLOSE_SHOOTER_RPM
                        + ratio * (MID_SHOOTER_RPM - CLOSE_SHOOTER_RPM);
            } else if (target.rangeInches <= LONG_DISTANCE_INCHES) {
                double ratio = Range.clip(
                        (target.rangeInches - MID_DISTANCE_INCHES)
                                / (LONG_DISTANCE_INCHES - MID_DISTANCE_INCHES),
                        0.0, 1.0);
                targetRpm = MID_SHOOTER_RPM
                        + ratio * (LONG_SHOOTER_RPM - MID_SHOOTER_RPM);
            } else {
                double ratio = Range.clip(
                        (target.rangeInches - LONG_DISTANCE_INCHES)
                                / (FAR_DISTANCE_INCHES - LONG_DISTANCE_INCHES),
                        0.0, 1.0);
                targetRpm = LONG_SHOOTER_RPM
                        + ratio * (FAR_SHOOTER_RPM - LONG_SHOOTER_RPM);
            }
            double ticksPerRevolution = launcher == null
                    ? 1.0 : launcher.getMotorType().getTicksPerRev();
            launcherTargetVelocity = targetRpm * ticksPerRevolution / 60.0;
        }
    }

    @Override
    protected boolean shouldRunLauncher() {
        return true;
    }

    @Override
    protected boolean showRedWhenNotReady() {
        return true;
    }

    @Override
    protected double windmillFeedReadyFraction() {
        // Red/Blue TeleOp keep shooter speed automatic, but both bumpers are
        // direct manual windmill controls with no velocity gate.
        return 0.0;
    }

    @Override
    protected double windmillFeedPauseFraction() {
        return 0.0;
    }

    @Override
    protected double leftBumperWindmillPower() {
        return -super.leftBumperWindmillPower();
    }

    @Override
    protected double leftTriggerWindmillPower(double trigger) {
        // In Red/Blue TeleOp the triggers control only the intake.
        return 0.0;
    }

    @Override
    protected double assistedRotation(double manualRotation) {
        if (!gamepad2.a || Math.abs(manualRotation) > 0.05 || !target.hasRange) {
            filteredAimErrorDegrees = Double.NaN;
            return manualRotation;
        }

        double desiredBearing = target.desiredBearingDegrees;
        double rawBearingError = target.bearingDegrees - desiredBearing;
        filteredAimErrorDegrees = Double.isNaN(filteredAimErrorDegrees)
                ? rawBearingError
                : AUTO_AIM_FILTER_ALPHA * rawBearingError
                + (1.0 - AUTO_AIM_FILTER_ALPHA) * filteredAimErrorDegrees;
        if (Math.abs(filteredAimErrorDegrees) <= AUTO_AIM_TOLERANCE_DEGREES) {
            return 0.0;
        }

        // Positive camera bearing requires the opposite drivetrain rotation
        // sign from the manual right-stick convention.
        double absoluteError = Math.abs(filteredAimErrorDegrees);
        double gain = absoluteError > 15.0 ? AUTO_AIM_FAR_KP
                : absoluteError > 5.0 ? AUTO_AIM_MID_KP
                : AUTO_AIM_NEAR_KP;
        double rotation = Range.clip(-filteredAimErrorDegrees * gain,
                -AUTO_AIM_MAX_POWER, AUTO_AIM_MAX_POWER);
        if (Math.abs(rotation) < AUTO_AIM_MIN_POWER) {
            rotation = Math.copySign(AUTO_AIM_MIN_POWER, rotation);
        }
        return rotation;
    }

    @Override
    protected boolean showAprilTags() {
        // Refresh after the launcher command so readiness uses the latest pose
        // and measured flywheel velocity.
        target = findBestRedScoringTarget();
        double actualVelocity = launcher == null ? 0.0 : Math.abs(launcher.getVelocity());
        double ticksPerRevolution = launcher == null
                ? 1.0 : launcher.getMotorType().getTicksPerRev();
        double targetRpm = launcherTargetVelocity * 60.0 / ticksPerRevolution;
        double actualRpm = actualVelocity * 60.0 / ticksPerRevolution;
        boolean atSpeed = launcher != null
                && actualVelocity >= READY_SPEED_FRACTION * launcherTargetVelocity;
        double minimumReadyDistance = minimumReadyDistance(target);
        boolean distanceReady = target.hasRange
                && target.rangeInches >= minimumReadyDistance;
        boolean instantaneousReady = target.angleReady && distanceReady && atSpeed;
        long now = System.nanoTime();
        if (instantaneousReady != pendingReadyToShoot) {
            pendingReadyToShoot = instantaneousReady;
            pendingReadySinceNanos = now;
        }
        double confirmSeconds = pendingReadyToShoot
                ? GREEN_CONFIRM_SECONDS : RED_CONFIRM_SECONDS;
        if (displayedReadyToShoot != pendingReadyToShoot
                && (now - pendingReadySinceNanos) / 1_000_000_000.0 >= confirmSeconds) {
            displayedReadyToShoot = pendingReadyToShoot;
        }
        boolean readyToShoot = displayedReadyToShoot;
        boolean feedRequested = gamepad2.right_bumper;

        if (readyToShoot && !lastReadyToShoot) {
            RobotLog.ii(TAG, "READY_TRANSITION alliance=%s red_to_green",
                    teleOpAllianceName());
        } else if (!readyToShoot && lastReadyToShoot) {
            RobotLog.ii(TAG, "READY_TRANSITION alliance=%s green_to_red",
                    teleOpAllianceName());
        }
        lastReadyToShoot = readyToShoot;
        if (feedRequested && !lastFeedRequested) {
            RobotLog.ii(TAG,
                    "SHOT range=%.1f minRange=%.1f targetRpm=%.0f actualRpm=%.0f bearing=%.1f bearingTarget=%.1f bearingMin=%.1f bearingMax=%.1f yawError=%.1f yawLimit=%.1f roll=%.1f ready=%d",
                    target.rangeInches, minimumReadyDistance, targetRpm, actualRpm,
                    target.bearingDegrees, target.desiredBearingDegrees,
                    target.allowedMinBearingDegrees, target.allowedMaxBearingDegrees,
                    target.yawFaceErrorDegrees, target.allowedYawErrorDegrees,
                    target.rollDegrees, readyToShoot ? 1 : 0);
            if (odometryAvailable) {
                RobotLog.ii(TAG,
                        "MAP_SHOT odoX=%.2f odoY=%.2f odoHeading=%.1f mapReady=%d mapRange=%.1f mapBearing=%.1f targetX=%.2f targetY=%.2f",
                        odometryX, odometryY, Math.toDegrees(odometryHeading),
                        targetMapReady ? 1 : 0, mappedRange(), mappedBearingDegrees(),
                        mappedTargetX, mappedTargetY);
            }
        }
        lastFeedRequested = feedRequested;

        telemetry.addData(teleOpAllianceName() + " target",
                target.visible ? target.name : "NOT VISIBLE");
        if (target.hasRange) {
            telemetry.addData("Target distance", "%.1f inches", target.rangeInches);
            telemetry.addData("Target angles", "bearing %+.1f, yaw %+.1f, roll %+.1f deg",
                    target.bearingDegrees, target.yawDegrees, target.rollDegrees);
            telemetry.addData("Yaw", "face error %.1f deg (informational)",
                    target.yawFaceErrorDegrees);
            telemetry.addData("Bearing calibration", "%+.1f <= %+.1f <= %+.1f deg; target %+.1f",
                    target.allowedMinBearingDegrees, target.bearingDegrees,
                    target.allowedMaxBearingDegrees, target.desiredBearingDegrees);
            telemetry.addData("Target approach angle", "camera %+.1f / mirrored calibration %+.1f deg",
                    target.rollDegrees, target.calibrationApproachDegrees);
            telemetry.addData("Minimum shooting distance", "%.1f inches (adjusted for target angle)",
                    minimumReadyDistance);
            telemetry.addData("A-button aim assist", "hold gp2 A; bearing error %+.1f deg",
                    target.bearingDegrees - target.desiredBearingDegrees);
        }
        telemetry.addData("Shooter readiness", "%.0f / %.0f ticks/s; %s",
                actualVelocity, launcherTargetVelocity,
                readyToShoot ? "READY - PRESS RB TO FEED" : "NOT READY");
        telemetry.addData("Shooter RPM", "actual %.0f / target %.0f",
                actualRpm, targetRpm);
        telemetry.addData("RPM calibration", "%.0fin=%.1f, %.0fin=%.1f, %.0fin=%.1f, %.0fin=%.1f",
                CLOSE_DISTANCE_INCHES, CLOSE_SHOOTER_RPM,
                MID_DISTANCE_INCHES, MID_SHOOTER_RPM,
                LONG_DISTANCE_INCHES, LONG_SHOOTER_RPM,
                FAR_DISTANCE_INCHES, FAR_SHOOTER_RPM);
        return readyToShoot;
    }

    private double minimumReadyDistance(TargetObservation observation) {
        return observation.hasRange
                ? interpolateApproachAngle(observation.calibrationApproachDegrees,
                NEGATIVE_45_MIN_DISTANCE_INCHES, ZERO_MIN_DISTANCE_INCHES,
                POSITIVE_45_MIN_DISTANCE_INCHES)
                : ZERO_MIN_DISTANCE_INCHES;
    }

    private static double interpolateApproachAngle(double angleDegrees,
                                                    double negative45Value,
                                                    double zeroValue,
                                                    double positive45Value) {
        double clampedAngle = Range.clip(angleDegrees, -45.0, 45.0);
        if (clampedAngle < 0.0) {
            double ratio = (clampedAngle + 45.0) / 45.0;
            return negative45Value + ratio * (zeroValue - negative45Value);
        }
        double ratio = clampedAngle / 45.0;
        return zeroValue + ratio * (positive45Value - zeroValue);
    }

    private void resetOdometryAndMap() {
        if (!odometryAvailable) return;
        previousLeftTicks = leftPod.getCurrentPosition();
        previousRightTicks = rightPod.getCurrentPosition();
        previousCenterTicks = -centerPod.getCurrentPosition();
        odometryX = 0.0;
        odometryY = 0.0;
        odometryHeading = 0.0;
        targetMapReady = false;
        mapAnchorFrames = 0;
        mapAnchorTargetXSum = 0.0;
        mapAnchorTargetYSum = 0.0;
        mappedTargetX = Double.NaN;
        mappedTargetY = Double.NaN;
        RobotLog.ii(TAG, "MAP_RESET start at farthest 0-degree shooting position");
    }

    private void updateOdometry() {
        if (!odometryAvailable) return;
        int currentLeft = leftPod.getCurrentPosition();
        int currentRight = rightPod.getCurrentPosition();
        int currentCenter = -centerPod.getCurrentPosition();
        double deltaLeft = (currentLeft - previousLeftTicks) / LEFT_TICKS_PER_INCH;
        double deltaRight = (currentRight - previousRightTicks) / RIGHT_TICKS_PER_INCH;
        double deltaCenter = (currentCenter - previousCenterTicks) / CENTER_TICKS_PER_INCH;
        previousLeftTicks = currentLeft;
        previousRightTicks = currentRight;
        previousCenterTicks = currentCenter;

        double deltaHeading = (deltaLeft - deltaRight) / POD_SPACING_INCHES;
        double forward = (deltaLeft + deltaRight) / 2.0;
        double lateral = deltaCenter - CENTER_POD_X_INCHES * deltaHeading;
        double midpointHeading = odometryHeading + deltaHeading / 2.0;
        odometryX += forward * Math.cos(midpointHeading)
                - lateral * Math.sin(midpointHeading);
        odometryY += forward * Math.sin(midpointHeading)
                + lateral * Math.cos(midpointHeading);
        odometryHeading = normalizeRadians(odometryHeading + deltaHeading);
    }

    private void updateTargetMap() {
        if (!odometryAvailable) return;
        boolean resetPressed = gamepad2.back;
        if (resetPressed && !lastMapResetButton) resetOdometryAndMap();
        lastMapResetButton = resetPressed;

        if (target.hasRange) {
            double bearingRadians = Math.toRadians(target.bearingDegrees);
            double relativeForward = target.rangeInches * Math.cos(bearingRadians);
            double relativeLateral = target.rangeInches * Math.sin(bearingRadians);
            double cosHeading = Math.cos(odometryHeading);
            double sinHeading = Math.sin(odometryHeading);
            double observedTargetX = odometryX
                    + relativeForward * cosHeading - relativeLateral * sinHeading;
            double observedTargetY = odometryY
                    + relativeForward * sinHeading + relativeLateral * cosHeading;

            if (!targetMapReady) {
                mapAnchorTargetXSum += observedTargetX;
                mapAnchorTargetYSum += observedTargetY;
                mapAnchorFrames++;
                if (mapAnchorFrames >= MAP_ANCHOR_FRAMES) {
                    mappedTargetX = mapAnchorTargetXSum / mapAnchorFrames;
                    mappedTargetY = mapAnchorTargetYSum / mapAnchorFrames;
                    targetMapReady = true;
                    RobotLog.ii(TAG, "MAP_ANCHORED frames=%d target=(%.2f,%.2f)",
                            mapAnchorFrames, mappedTargetX, mappedTargetY);
                }
            } else {
                double correctionError = Math.hypot(
                        observedTargetX - mappedTargetX,
                        observedTargetY - mappedTargetY);
                if (correctionError <= MAP_MAX_CORRECTION_ERROR_INCHES) {
                    mappedTargetX += MAP_CORRECTION_ALPHA
                            * (observedTargetX - mappedTargetX);
                    mappedTargetY += MAP_CORRECTION_ALPHA
                            * (observedTargetY - mappedTargetY);
                }
            }
        }

        telemetry.addData("Odometry map", targetMapReady
                ? "READY (gp2 BACK resets)" : "ANCHORING %d/%d - hold still",
                mapAnchorFrames, MAP_ANCHOR_FRAMES);
        telemetry.addData("Odometry pose", "x %.1f, y %.1f, heading %+.1f deg",
                odometryX, odometryY, Math.toDegrees(odometryHeading));
        if (targetMapReady) {
            telemetry.addData("Mapped target", "range %.1f, bearing %+.1f deg",
                    mappedRange(), mappedBearingDegrees());
            telemetry.addData("Camera vs map", "range %.1f / %.1f, bearing %+.1f / %+.1f",
                    target.rangeInches, mappedRange(), target.bearingDegrees,
                    mappedBearingDegrees());
        }
    }

    private double mappedRange() {
        if (!targetMapReady) return Double.NaN;
        return Math.hypot(mappedTargetX - odometryX, mappedTargetY - odometryY);
    }

    private double mappedBearingDegrees() {
        if (!targetMapReady) return Double.NaN;
        double deltaX = mappedTargetX - odometryX;
        double deltaY = mappedTargetY - odometryY;
        double relativeForward = Math.cos(odometryHeading) * deltaX
                + Math.sin(odometryHeading) * deltaY;
        double relativeLateral = -Math.sin(odometryHeading) * deltaX
                + Math.cos(odometryHeading) * deltaY;
        return Math.toDegrees(Math.atan2(relativeLateral, relativeForward));
    }

    private static double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private TargetObservation findBestRedScoringTarget() {
        if (aprilTags == null || webcamPortal == null) {
            return TargetObservation.none();
        }

        TargetObservation best = TargetObservation.none();
        for (AprilTagDetection detection : aprilTags.getDetections()) {
            boolean allianceTarget = false;
            boolean mirroredTarget = false;
            String name = defaultTargetName();
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                name = cluster.metadata == null ? "unknown cluster" : cluster.metadata.name;
                // The SDK cluster solver returns the cluster-origin pose, not
                // the center of whichever member tag happened to be visible.
                // Its registered member offsets reconstruct the same physical
                // target center at 25%, 50%, 75%, or 100% visibility.
                allianceTarget = isTeleOpTargetCluster(name)
                        && cluster.percentClusterFound >= MIN_CLUSTER_VISIBILITY_PERCENT;
                mirroredTarget = isMirroredTargetCluster(name);
            } else if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                allianceTarget = isTeleOpTargetSingleTag(single.id);
                mirroredTarget = isMirroredTargetSingleTag(single.id);
                name = single.metadata == null
                        ? targetNameForSingleTag(single.id) + " tag " + single.id
                        : single.metadata.name;
            }

            if (!allianceTarget || detection.ftcPose == null
                    || !Double.isFinite(detection.ftcPose.range)
                    || detection.ftcPose.range <= 0.0) {
                continue;
            }

            TargetObservation candidate = new TargetObservation(name,
                    detection.ftcPose.range, detection.ftcPose.bearing,
                    detection.ftcPose.yaw, detection.ftcPose.roll,
                    mirroredTarget);
            if (!best.hasRange
                    || Math.abs(candidate.bearingDegrees) < Math.abs(best.bearingDegrees)) {
                best = candidate;
            }
        }
        return best;
    }

    private TargetObservation findTargetWithMemory() {
        TargetObservation liveTarget = findBestRedScoringTarget();
        long now = System.nanoTime();
        if (liveTarget.hasRange) {
            lastValidTarget = liveTarget;
            lastValidTargetNanos = now;
            return liveTarget;
        }
        double ageSeconds = (now - lastValidTargetNanos) / 1_000_000_000.0;
        if (lastValidTarget.hasRange && ageSeconds <= TARGET_MEMORY_SECONDS) {
            return lastValidTarget;
        }
        return TargetObservation.none();
    }

    @Override
    protected String controlsDescription() {
        return "RED TELEOP: shooter auto speed; RB feeds; hold gp2 A for nearby aim assist";
    }

    protected String defaultTargetName() {
        return "RED SCORING";
    }

    protected String teleOpAllianceName() {
        return "Red";
    }

    protected boolean isTeleOpTargetCluster(String name) {
        return name.equalsIgnoreCase("RED SCORING")
                || name.equalsIgnoreCase("RED AUDIENCE");
    }

    protected boolean isMirroredTargetCluster(String name) {
        return name.equalsIgnoreCase("RED AUDIENCE");
    }

    protected boolean isTeleOpTargetSingleTag(int id) {
        return id >= 30 && id <= 37;
    }

    protected boolean isMirroredTargetSingleTag(int id) {
        return id >= 34 && id <= 37;
    }

    protected String targetNameForSingleTag(int id) {
        return id <= 33 ? "RED SCORING" : "RED AUDIENCE";
    }

    private static final class TargetObservation {
        final boolean visible;
        final boolean hasRange;
        final String name;
        final double rangeInches;
        final double bearingDegrees;
        final double yawDegrees;
        final double rollDegrees;
        final double calibrationApproachDegrees;
        final double yawFaceErrorDegrees;
        final double allowedYawErrorDegrees;
        final double desiredBearingDegrees;
        final double allowedMinBearingDegrees;
        final double allowedMaxBearingDegrees;
        final boolean angleReady;

        TargetObservation(String name, double rangeInches, double bearingDegrees,
                          double yawDegrees, double rollDegrees,
                          boolean mirroredTarget) {
            this.visible = true;
            this.hasRange = true;
            this.name = name;
            this.rangeInches = rangeInches;
            this.bearingDegrees = bearingDegrees;
            this.yawDegrees = yawDegrees;
            this.rollDegrees = rollDegrees;
            this.calibrationApproachDegrees = mirroredTarget
                    ? -rollDegrees : rollDegrees;
            double wrappedYaw = Math.abs(yawDegrees) % 360.0;
            if (wrappedYaw > 180.0) wrappedYaw = 360.0 - wrappedYaw;
            this.yawFaceErrorDegrees = Math.min(wrappedYaw,
                    Math.abs(180.0 - wrappedYaw));
            double distanceRatio = Range.clip(
                    (rangeInches - CLOSE_DISTANCE_INCHES)
                            / (FAR_DISTANCE_INCHES - CLOSE_DISTANCE_INCHES),
                    0.0, 1.0);
            this.allowedYawErrorDegrees = NEAR_MAX_YAW_ERROR_DEGREES
                    + distanceRatio * (FAR_MAX_YAW_ERROR_DEGREES
                    - NEAR_MAX_YAW_ERROR_DEGREES);
            double baseDesiredBearing = interpolateApproachAngle(
                    calibrationApproachDegrees,
                    NEGATIVE_45_TARGET_BEARING_DEGREES, ZERO_TARGET_BEARING_DEGREES,
                    POSITIVE_45_TARGET_BEARING_DEGREES);
            this.desiredBearingDegrees = mirroredTarget
                    ? -baseDesiredBearing : baseDesiredBearing;
            double baseLeftTolerance = interpolateApproachAngle(
                    calibrationApproachDegrees,
                    NEGATIVE_45_LEFT_TOLERANCE_DEGREES,
                    ZERO_LEFT_TOLERANCE_DEGREES,
                    POSITIVE_45_LEFT_TOLERANCE_DEGREES);
            double baseRightTolerance = interpolateApproachAngle(
                    calibrationApproachDegrees,
                    NEGATIVE_45_RIGHT_TOLERANCE_DEGREES,
                    ZERO_RIGHT_TOLERANCE_DEGREES,
                    POSITIVE_45_RIGHT_TOLERANCE_DEGREES);
            double leftTolerance = mirroredTarget
                    ? baseRightTolerance : baseLeftTolerance;
            double rightTolerance = mirroredTarget
                    ? baseLeftTolerance : baseRightTolerance;
            this.allowedMinBearingDegrees = desiredBearingDegrees - leftTolerance;
            this.allowedMaxBearingDegrees = desiredBearingDegrees + rightTolerance;
            this.angleReady = bearingDegrees >= allowedMinBearingDegrees
                    && bearingDegrees <= allowedMaxBearingDegrees;
        }

        private TargetObservation() {
            visible = false;
            hasRange = false;
            name = "NONE";
            rangeInches = Double.NaN;
            bearingDegrees = Double.NaN;
            yawDegrees = Double.NaN;
            rollDegrees = Double.NaN;
            calibrationApproachDegrees = Double.NaN;
            yawFaceErrorDegrees = Double.NaN;
            allowedYawErrorDegrees = Double.NaN;
            desiredBearingDegrees = Double.NaN;
            allowedMinBearingDegrees = Double.NaN;
            allowedMaxBearingDegrees = Double.NaN;
            angleReady = false;
        }

        static TargetObservation none() {
            return new TargetObservation();
        }
    }
}
