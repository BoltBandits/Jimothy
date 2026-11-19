package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private static final double MIN_READY_BEARING_DEGREES = -7.0;
    private static final double MAX_READY_BEARING_DEGREES = 5.5;
    // Closer than this, the fixed launcher elevation is too flat to clear the
    // lower rim reliably even when wheel speed and horizontal aim are correct.
    private static final double MIN_READY_DISTANCE_INCHES = 24.0;
    private static final double MAX_ANGLED_DISTANCE_ADDITION_INCHES = 3.0;
    private static final double FULL_DISTANCE_ADDITION_ROLL_DEGREES = 45.0;
    private static final double NEAR_MAX_YAW_ERROR_DEGREES = 50.0;
    private static final double FAR_MAX_YAW_ERROR_DEGREES = 20.0;
    private static final double BEARING_NARROWING_YAW_DEGREES = 45.0;
    private static final double BEARING_WIDTH_AT_45_DEGREES = 0.40;
    // At an oblique approach only one of the four scoring tags may be visible.
    // The cluster identity still guarantees it belongs to the red target.
    private static final int MIN_CLUSTER_VISIBILITY_PERCENT = 25;
    private static final double READY_SPEED_FRACTION = 0.97;
    private static final double TARGET_MEMORY_SECONDS = 3.0;
    private static final double GREEN_CONFIRM_SECONDS = 0.12;
    private static final double RED_CONFIRM_SECONDS = 0.25;

    private TargetObservation target = TargetObservation.none();
    private TargetObservation lastValidTarget = TargetObservation.none();
    private long lastValidTargetNanos;
    private boolean lastReadyToShoot;
    private boolean pendingReadyToShoot;
    private boolean displayedReadyToShoot;
    private long pendingReadySinceNanos;
    private boolean lastFeedRequested;

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
        return 0.99;
    }

    @Override
    protected double windmillFeedPauseFraction() {
        return 0.97;
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
        double minimumReadyDistance = MIN_READY_DISTANCE_INCHES;
        if (target.hasRange) {
            double rollRatio = Range.clip(
                    Math.abs(target.rollDegrees) / FULL_DISTANCE_ADDITION_ROLL_DEGREES,
                    0.0, 1.0);
            minimumReadyDistance += rollRatio * MAX_ANGLED_DISTANCE_ADDITION_INCHES;
        }
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
            gamepad2.rumbleBlips(2);
        }
        lastReadyToShoot = readyToShoot;
        if (feedRequested && !lastFeedRequested) {
            RobotLog.ii(TAG,
                    "SHOT range=%.1f minRange=%.1f targetRpm=%.0f actualRpm=%.0f bearing=%.1f bearingMin=%.1f bearingMax=%.1f yawError=%.1f yawLimit=%.1f roll=%.1f ready=%d",
                    target.rangeInches, minimumReadyDistance, targetRpm, actualRpm,
                    target.bearingDegrees,
                    target.allowedMinBearingDegrees, target.allowedMaxBearingDegrees,
                    target.yawFaceErrorDegrees, target.allowedYawErrorDegrees,
                    target.rollDegrees, readyToShoot ? 1 : 0);
        }
        lastFeedRequested = feedRequested;

        telemetry.addData("Red target", target.visible ? target.name : "NOT VISIBLE");
        if (target.hasRange) {
            telemetry.addData("Target distance", "%.1f inches", target.rangeInches);
            telemetry.addData("Target angles", "bearing %+.1f, yaw %+.1f, roll %+.1f deg",
                    target.bearingDegrees, target.yawDegrees, target.rollDegrees);
            telemetry.addData("Yaw gate", "face error %.1f <= %.1f deg at %.1fin",
                    target.yawFaceErrorDegrees, target.allowedYawErrorDegrees,
                    target.rangeInches);
            telemetry.addData("Bearing gate", "%+.1f <= %+.1f <= %+.1f deg (narrows with yaw)",
                    target.allowedMinBearingDegrees, target.bearingDegrees,
                    target.allowedMaxBearingDegrees);
            telemetry.addData("Target approach angle", "%.1f deg absolute roll",
                    Math.abs(target.rollDegrees));
            telemetry.addData("Minimum shooting distance", "%.1f inches (adjusted for target angle)",
                    minimumReadyDistance);
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

    private TargetObservation findBestRedScoringTarget() {
        if (aprilTags == null || webcamPortal == null) {
            return TargetObservation.none();
        }

        TargetObservation best = TargetObservation.none();
        for (AprilTagDetection detection : aprilTags.getDetections()) {
            boolean redScoring = false;
            String name = "RED SCORING";
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                name = cluster.metadata == null ? "unknown cluster" : cluster.metadata.name;
                redScoring = name.equalsIgnoreCase("RED SCORING")
                        && cluster.percentClusterFound >= MIN_CLUSTER_VISIBILITY_PERCENT;
            } else if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                redScoring = single.id >= 30 && single.id <= 33;
                name = single.metadata == null ? "RED SCORING tag " + single.id
                        : single.metadata.name;
            }

            if (!redScoring || detection.ftcPose == null
                    || !Double.isFinite(detection.ftcPose.range)
                    || detection.ftcPose.range <= 0.0) {
                continue;
            }

            TargetObservation candidate = new TargetObservation(name,
                    detection.ftcPose.range, detection.ftcPose.bearing,
                    detection.ftcPose.yaw, detection.ftcPose.roll);
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
        return "RED TELEOP: shooter always on/auto speed; RB feeds when green";
    }

    private static final class TargetObservation {
        final boolean visible;
        final boolean hasRange;
        final String name;
        final double rangeInches;
        final double bearingDegrees;
        final double yawDegrees;
        final double rollDegrees;
        final double yawFaceErrorDegrees;
        final double allowedYawErrorDegrees;
        final double allowedMinBearingDegrees;
        final double allowedMaxBearingDegrees;
        final boolean angleReady;

        TargetObservation(String name, double rangeInches, double bearingDegrees,
                          double yawDegrees, double rollDegrees) {
            this.visible = true;
            this.hasRange = true;
            this.name = name;
            this.rangeInches = rangeInches;
            this.bearingDegrees = bearingDegrees;
            this.yawDegrees = yawDegrees;
            this.rollDegrees = rollDegrees;
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
            double yawNarrowingRatio = Range.clip(
                    yawFaceErrorDegrees / BEARING_NARROWING_YAW_DEGREES,
                    0.0, 1.0);
            double bearingScale = 1.0 - yawNarrowingRatio
                    * (1.0 - BEARING_WIDTH_AT_45_DEGREES);
            double bearingCenter = (MIN_READY_BEARING_DEGREES
                    + MAX_READY_BEARING_DEGREES) / 2.0;
            double bearingHalfWidth = (MAX_READY_BEARING_DEGREES
                    - MIN_READY_BEARING_DEGREES) / 2.0 * bearingScale;
            this.allowedMinBearingDegrees = bearingCenter - bearingHalfWidth;
            this.allowedMaxBearingDegrees = bearingCenter + bearingHalfWidth;
            this.angleReady = bearingDegrees >= allowedMinBearingDegrees
                    && bearingDegrees <= allowedMaxBearingDegrees
                    && yawFaceErrorDegrees <= allowedYawErrorDegrees;
        }

        private TargetObservation() {
            visible = false;
            hasRange = false;
            name = "NONE";
            rangeInches = Double.NaN;
            bearingDegrees = Double.NaN;
            yawDegrees = Double.NaN;
            rollDegrees = Double.NaN;
            yawFaceErrorDegrees = Double.NaN;
            allowedYawErrorDegrees = Double.NaN;
            allowedMinBearingDegrees = Double.NaN;
            allowedMaxBearingDegrees = Double.NaN;
            angleReady = false;
        }

        static TargetObservation none() {
            return new TargetObservation();
        }
    }
}
