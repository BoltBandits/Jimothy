package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "Shooter Velocity Test", group = "Jimothy Calibration")
public class ShooterVelocityTest extends OpMode {
    private static final String TAG = "ShooterVelocityTest";
    // Match the close-position Red/Blue TeleOp command exactly so unloaded
    // vibration at this specific speed can be isolated from windmill loading.
    private static final double NORMAL_TARGET_VELOCITY = 1347.0;
    private static final double TURBO_SPEED_FRACTION = 0.6179976;
    private static final double READY_FRACTION = 0.97;
    private static final double WINDMILL_POWER = -0.25;
    private static final double SHOOTER_P = 40.0;
    private static final double SHOOTER_F = 12.5;
    private static final double LOG_PERIOD_SECONDS = 0.05;

    private DcMotorEx shooter;
    private CRServo windmill;
    private double maxVelocity;
    private double targetVelocity;
    private boolean lastFeedButton;
    private boolean measuringShot;
    private double preFeedVelocity;
    private double minimumVelocity;
    private double velocityDrop;
    private double velocityDropPercent;
    private double recoverySeconds;
    private boolean hasResult;
    private long lastLoopNanos;
    private double worstLoopGapMs;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        windmill = hardwareMap.get(CRServo.class, "windmill_servo");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, 0.0, 0.0, SHOOTER_F));
        shooter.setVelocity(0.0);
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        windmill.setPower(0.0);

        maxVelocity = shooter.getMotorType().getMaxRPM()
                * shooter.getMotorType().getTicksPerRev() / 60.0;
        targetVelocity = NORMAL_TARGET_VELOCITY;
        telemetry.addLine("Either gamepad: RB normal | A turbo | X feed | B reset");
        telemetry.addLine("Secure the robot and keep clear of the shooter.");
    }

    @Override
    public void start() {
        lastLoopNanos = System.nanoTime();
        worstLoopGapMs = 0.0;
    }

    @Override
    public void loop() {
        long nowNanos = System.nanoTime();
        double loopGapMs = (nowNanos - lastLoopNanos) / 1_000_000.0;
        lastLoopNanos = nowNanos;
        worstLoopGapMs = Math.max(worstLoopGapMs, loopGapMs);

        boolean turbo = gamepad1.a || gamepad2.a;
        boolean shooterRequested = gamepad1.right_bumper || gamepad2.right_bumper || turbo;
        boolean feedRequested = (gamepad1.x || gamepad2.x) && shooterRequested;
        targetVelocity = turbo
                ? TURBO_SPEED_FRACTION * maxVelocity
                : NORMAL_TARGET_VELOCITY;
        shooter.setVelocity(shooterRequested ? targetVelocity : 0.0);
        windmill.setPower(feedRequested ? WINDMILL_POWER : 0.0);

        double actualVelocity = Math.abs(shooter.getVelocity());
        if (feedRequested && !lastFeedButton) {
            measuringShot = true;
            hasResult = false;
            preFeedVelocity = actualVelocity;
            minimumVelocity = actualVelocity;
            shotTimer.reset();
            RobotLog.ii(TAG, "SHOT_START target=%.0f pre=%.0f", targetVelocity, preFeedVelocity);
        }

        if (measuringShot) {
            minimumVelocity = Math.min(minimumVelocity, actualVelocity);
            if (!feedRequested && actualVelocity >= READY_FRACTION * targetVelocity) {
                measuringShot = false;
                hasResult = true;
                velocityDrop = Math.max(0.0, preFeedVelocity - minimumVelocity);
                velocityDropPercent = preFeedVelocity > 0.0
                        ? 100.0 * velocityDrop / preFeedVelocity : 0.0;
                recoverySeconds = shotTimer.seconds();
                RobotLog.ii(TAG,
                        "SHOT_RESULT target=%.0f pre=%.0f min=%.0f drop=%.0f dropPct=%.1f recovery=%.3f",
                        targetVelocity, preFeedVelocity, minimumVelocity, velocityDrop,
                        velocityDropPercent, recoverySeconds);
            }
        }

        if (gamepad1.b || gamepad2.b) {
            measuringShot = false;
            hasResult = false;
            worstLoopGapMs = 0.0;
        }
        lastFeedButton = feedRequested;

        if (logTimer.seconds() >= LOG_PERIOD_SECONDS) {
            logTimer.reset();
            RobotLog.ii(TAG, "target=%.0f actual=%.0f shooter=%d feed=%d loopMs=%.1f",
                    shooterRequested ? targetVelocity : 0.0, actualVelocity,
                    shooterRequested ? 1 : 0, feedRequested ? 1 : 0, loopGapMs);
        }

        telemetry.addLine("Either gamepad: RB normal | A turbo | X feed | B reset");
        telemetry.addData("Shooter", shooterRequested ? (turbo ? "TURBO" : "NORMAL") : "OFF");
        telemetry.addData("Velocity target / actual", "%.0f / %.0f ticks/s", targetVelocity, actualVelocity);
        telemetry.addData("At feed speed", actualVelocity >= READY_FRACTION * targetVelocity ? "YES" : "NO");
        telemetry.addData("Windmill", feedRequested ? "FEEDING at %.2f" : "STOPPED", WINDMILL_POWER);
        telemetry.addData("Loop gap current / worst", "%.1f / %.1f ms", loopGapMs, worstLoopGapMs);
        if (measuringShot) {
            telemetry.addData("Shot measurement", "RUNNING: pre %.0f, min %.0f",
                    preFeedVelocity, minimumVelocity);
        } else if (hasResult) {
            telemetry.addData("Last shot pre / min", "%.0f / %.0f ticks/s",
                    preFeedVelocity, minimumVelocity);
            telemetry.addData("Last shot drop", "%.0f ticks/s (%.1f%%)",
                    velocityDrop, velocityDropPercent);
            telemetry.addData("Recovery to 97%%", "%.3f s", recoverySeconds);
        } else {
            telemetry.addData("Shot measurement", "No result yet");
        }
    }

    @Override
    public void stop() {
        windmill.setPower(0.0);
        shooter.setVelocity(0.0);
    }
}
