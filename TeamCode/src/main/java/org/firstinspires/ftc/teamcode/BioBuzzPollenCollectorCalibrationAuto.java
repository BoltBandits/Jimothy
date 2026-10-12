package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;
import java.util.Locale;

/**
 * Experimental pollen-seeking autonomous. Run with the robot on blocks first.
 *
 * Distance is initially estimated from detector area. Use the generated CSV to tune
 * AREA_DISTANCE_SCALE_INCHES_SQRT_PERCENT, then add the measured camera forward offset.
 */
@Autonomous(name = "Jimothy Pollen Collector Calibration", group = "Diagnostics")
public class BioBuzzPollenCollectorCalibrationAuto extends LinearOpMode {
    private static final String TAG = "PollenCollector";
    private static final int LIMELIGHT_PIPELINE = 0;

    // Measured camera location. Positive lateral offset is to the robot's right.
    private static final double CAMERA_RIGHT_INCHES = 6.5;
    private static final double CAMERA_HEIGHT_INCHES = 9.5;
    // Measure from robot center to camera lens; positive is forward.
    private static final double CAMERA_FORWARD_INCHES = 0.0;

    // First-pass distance model: distance = scale / sqrt(target area percent).
    // Tune this from the CSV using known tape-measured distances.
    private static final double AREA_DISTANCE_SCALE_INCHES_SQRT_PERCENT = 30.0;
    private static final double MIN_USABLE_AREA_PERCENT = 0.03;
    private static final double MIN_CONFIDENCE = 0.35;

    private static final double MAX_FORWARD_POWER = 0.28;
    private static final double MIN_FORWARD_POWER = 0.10;
    private static final double MAX_TURN_POWER = 0.28;
    private static final double TURN_KP = 0.018;
    private static final double TURN_DEADBAND_DEGREES = 2.0;
    private static final double SLOW_AREA_PERCENT = 5.0;
    private static final double STOP_AREA_PERCENT = 12.0;
    private static final long MAX_RESULT_STALENESS_MS = 250;
    private static final double LOST_INTAKE_HOLD_SECONDS = 5.0;
    private static final double TREND_SAMPLE_SECONDS = 0.40;
    private static final double MAX_ALLOWED_AREA_DROP_FRACTION = 0.18;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intakeMotor;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private Limelight3A limelight;

    private PrintWriter dataLog;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime sincePollenSeen = new ElapsedTime();
    private final ElapsedTime trendTimer = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();
    private boolean hasSeenPollen;
    private double previousTrendArea = Double.NaN;
    private double areaRate;
    private boolean approaching = true;

    @Override
    public void runOpMode() {
        initializeHardware();
        openDataLog();

        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Pollen calibration auto ready");
        telemetry.addLine("CAUTION: starts driving when pollen is visible");
        telemetry.addData("Camera right / height", "%.1f / %.1f in",
                CAMERA_RIGHT_INCHES, CAMERA_HEIGHT_INCHES);
        telemetry.addData("Camera forward (needs measurement)", "%.1f in",
                CAMERA_FORWARD_INCHES);
        telemetry.addData("Distance scale (needs calibration)", "%.1f",
                AREA_DISTANCE_SCALE_INCHES_SQRT_PERCENT);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            shutdown();
            return;
        }

        runtime.reset();
        sincePollenSeen.reset();
        trendTimer.reset();
        logTimer.reset();

        try {
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                LLResultTypes.DetectorResult pollen = closestPollen(result);
                boolean fresh = result != null
                        && result.getStaleness() <= MAX_RESULT_STALENESS_MS;

                double forward = 0;
                double turn = 0;
                double estimatedDistance = Double.NaN;
                double correctedBearing = Double.NaN;
                String state;

                if (fresh && pollen != null) {
                    hasSeenPollen = true;
                    sincePollenSeen.reset();
                    setIntake(true);

                    double area = pollen.getTargetArea();
                    estimatedDistance = estimateDistance(area);
                    correctedBearing = correctedRobotBearing(
                            pollen.getTargetXDegrees(), estimatedDistance);
                    updateApproachTrend(area);

                    turn = turnCommand(correctedBearing);
                    if (area >= STOP_AREA_PERCENT) {
                        state = "POLLEN AT INTAKE - DRIVE STOPPED";
                    } else if (!approaching) {
                        state = "AREA DECREASED - REACQUIRING";
                    } else {
                        forward = forwardCommand(area, correctedBearing);
                        state = "TRACKING CLOSEST POLLEN";
                    }
                } else {
                    stopTrendTracking();
                    stopDrive();
                    boolean holdIntake = hasSeenPollen
                            && sincePollenSeen.seconds() < LOST_INTAKE_HOLD_SECONDS;
                    setIntake(holdIntake);
                    state = holdIntake
                            ? "POLLEN LOST - INTAKE 5 SEC HOLD"
                            : "SEARCHING - STOPPED";
                }

                // This drivetrain's configured motor directions require negative mix
                // for physical forward, matching the existing Jimothy autonomous.
                setMecanum(-forward, turn);
                showTelemetry(state, result, pollen, estimatedDistance,
                        correctedBearing, forward, turn);
                logSample(state, result, pollen, estimatedDistance,
                        correctedBearing, forward, turn);
                idle();
            }
        } finally {
            shutdown();
        }
    }

    private void initializeHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
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
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        setIntake(false);
        stopDrive();
    }

    private LLResultTypes.DetectorResult closestPollen(LLResult result) {
        if (result == null || result.getStaleness() > MAX_RESULT_STALENESS_MS) return null;
        LLResultTypes.DetectorResult closest = null;
        for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {
            String label = detection.getClassName();
            if (label == null || !label.toLowerCase(Locale.US).trim().equals("pollen")) continue;
            if (detection.getConfidence() < MIN_CONFIDENCE
                    || detection.getTargetArea() < MIN_USABLE_AREA_PERCENT) continue;
            // With one calibrated camera, the largest image area is the best available
            // proxy for the physically closest pollen.
            if (closest == null || detection.getTargetArea() > closest.getTargetArea()) {
                closest = detection;
            }
        }
        return closest;
    }

    private double estimateDistance(double areaPercent) {
        return AREA_DISTANCE_SCALE_INCHES_SQRT_PERCENT
                / Math.sqrt(Math.max(areaPercent, MIN_USABLE_AREA_PERCENT));
    }

    private double correctedRobotBearing(double cameraTxDegrees, double cameraDistance) {
        double cameraBearing = Math.toRadians(cameraTxDegrees);
        double targetRightFromCamera = cameraDistance * Math.sin(cameraBearing);
        double targetForwardFromCamera = cameraDistance * Math.cos(cameraBearing);
        double targetRightFromRobot = CAMERA_RIGHT_INCHES + targetRightFromCamera;
        double targetForwardFromRobot = CAMERA_FORWARD_INCHES + targetForwardFromCamera;
        return Math.toDegrees(Math.atan2(targetRightFromRobot, targetForwardFromRobot));
    }

    private void updateApproachTrend(double area) {
        if (trendTimer.seconds() < TREND_SAMPLE_SECONDS) return;
        if (Double.isNaN(previousTrendArea)) {
            areaRate = 0;
            approaching = true;
        } else {
            areaRate = (area - previousTrendArea) / trendTimer.seconds();
            approaching = area >= previousTrendArea * (1.0 - MAX_ALLOWED_AREA_DROP_FRACTION);
        }
        previousTrendArea = area;
        trendTimer.reset();
    }

    private void stopTrendTracking() {
        previousTrendArea = Double.NaN;
        areaRate = 0;
        approaching = true;
        trendTimer.reset();
    }

    private double turnCommand(double correctedBearingDegrees) {
        if (Math.abs(correctedBearingDegrees) <= TURN_DEADBAND_DEGREES) return 0;
        // Positive Limelight/robot bearing is right; positive mecanum turn is CCW/left.
        return Range.clip(-correctedBearingDegrees * TURN_KP,
                -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    private double forwardCommand(double area, double correctedBearingDegrees) {
        if (Math.abs(correctedBearingDegrees) > 35.0) return 0;
        double areaScale = Range.clip(
                (STOP_AREA_PERCENT - area) / (STOP_AREA_PERCENT - SLOW_AREA_PERCENT),
                0.0, 1.0);
        double headingScale = Range.clip(1.0 - Math.abs(correctedBearingDegrees) / 40.0,
                0.25, 1.0);
        return (MIN_FORWARD_POWER
                + (MAX_FORWARD_POWER - MIN_FORWARD_POWER) * areaScale) * headingScale;
    }

    private void setMecanum(double forward, double turn) {
        double fl = forward + turn;
        double fr = forward - turn;
        double bl = forward + turn;
        double br = forward - turn;
        double scale = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / scale);
        frontRight.setPower(fr / scale);
        backLeft.setPower(bl / scale);
        backRight.setPower(br / scale);
    }

    private void stopDrive() {
        setMecanum(0, 0);
    }

    private void setIntake(boolean enabled) {
        intakeMotor.setPower(enabled ? 1.0 : 0.0);
        leftIntakeServo.setPower(enabled ? -1.0 : 0.0);
        rightIntakeServo.setPower(enabled ? -1.0 : 0.0);
    }

    private void showTelemetry(String state, LLResult result,
                               LLResultTypes.DetectorResult pollen,
                               double distance, double bearing,
                               double forward, double turn) {
        LLStatus status = limelight.getStatus();
        telemetry.addData("State", state);
        telemetry.addData("Limelight", "pipeline %d, %.0f FPS",
                status.getPipelineIndex(), status.getFps());
        telemetry.addData("Result staleness", result == null ? "none" : "%d ms",
                result == null ? 0 : result.getStaleness());
        if (pollen != null) {
            telemetry.addData("Closest pollen", "conf %.0f%% area %.3f%%",
                    pollen.getConfidence() * 100.0, pollen.getTargetArea());
            telemetry.addData("Camera tx / ty", "%+.2f / %+.2f deg",
                    pollen.getTargetXDegrees(), pollen.getTargetYDegrees());
            telemetry.addData("Estimated distance", "%.1f in", distance);
            telemetry.addData("Offset-corrected bearing", "%+.2f deg", bearing);
            telemetry.addData("Area rate", "%+.3f %%/s", areaRate);
            telemetry.addData("Getting closer", approaching);
        }
        telemetry.addData("Drive forward / turn", "%+.2f / %+.2f", forward, turn);
        telemetry.addData("Intake", intakeMotor.getPower() != 0 ? "ON" : "OFF");
        telemetry.addData("Seconds since pollen", hasSeenPollen
                ? "%.2f" : "never", hasSeenPollen ? sincePollenSeen.seconds() : 0);
        telemetry.addData("Calibration CSV", "FIRST/settings/JimothyPollenCalibration.csv");
        telemetry.update();
    }

    private void openDataLog() {
        try {
            File file = AppUtil.getInstance().getSettingsFile("JimothyPollenCalibration.csv");
            boolean writeHeader = !file.exists() || file.length() == 0;
            dataLog = new PrintWriter(new FileWriter(file, true));
            if (writeHeader) {
                dataLog.println("time_s,state,staleness_ms,count,confidence,area_percent,tx_deg,ty_deg,estimated_distance_in,corrected_bearing_deg,area_rate_percent_s,forward,turn,intake");
            }
            dataLog.printf(Locale.US, "# new_run,camera_height=%.2f,camera_right=%.2f,camera_forward=%.2f,area_scale=%.2f%n",
                    CAMERA_HEIGHT_INCHES, CAMERA_RIGHT_INCHES, CAMERA_FORWARD_INCHES,
                    AREA_DISTANCE_SCALE_INCHES_SQRT_PERCENT);
            dataLog.flush();
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "Could not open pollen calibration CSV");
            dataLog = null;
        }
    }

    private void logSample(String state, LLResult result,
                           LLResultTypes.DetectorResult pollen,
                           double distance, double bearing,
                           double forward, double turn) {
        if (logTimer.milliseconds() < 100) return;
        logTimer.reset();
        int count = result == null ? 0 : result.getDetectorResults().size();
        long staleness = result == null ? -1 : result.getStaleness();
        double confidence = pollen == null ? Double.NaN : pollen.getConfidence();
        double area = pollen == null ? Double.NaN : pollen.getTargetArea();
        double tx = pollen == null ? Double.NaN : pollen.getTargetXDegrees();
        double ty = pollen == null ? Double.NaN : pollen.getTargetYDegrees();
        if (dataLog != null) {
            dataLog.printf(Locale.US,
                    "%.3f,%s,%d,%d,%.4f,%.5f,%.3f,%.3f,%.3f,%.3f,%.5f,%.3f,%.3f,%d%n",
                    runtime.seconds(), state.replace(',', ';'), staleness, count,
                    confidence, area, tx, ty, distance, bearing, areaRate,
                    forward, turn, intakeMotor.getPower() != 0 ? 1 : 0);
            dataLog.flush();
        }
        RobotLog.dd(TAG,
                "state=%s area=%.3f tx=%.2f ty=%.2f dist=%.1f bearing=%.2f forward=%.2f turn=%.2f",
                state, area, tx, ty, distance, bearing, forward, turn);
    }

    private void shutdown() {
        stopDrive();
        setIntake(false);
        if (limelight != null) limelight.stop();
        if (dataLog != null) {
            dataLog.flush();
            dataLog.close();
            dataLog = null;
        }
    }

    private void prepareDriveMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }
}
