package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * StarterBot TeleOp v6: same controls as v5, plus an AprilTag alignment light.
 *
 * The launcher runs at 51.4998% of the motor's full speed (v5 used 350 ticks/s).
 * Holding gamepad 2 A (Cross) shoots in TURBO at 61.79976% instead. Each time turbo
 * starts, the Driver Station says "Turbo" and gamepad 2 rumbles (the Control
 * Hub has no speaker of its own).
 *
 * The indicator lights are GREEN only while a RED SCORING or RED AUDIENCE
 * AprilTag is visible and upright within 10 degrees of roll. Otherwise every
 * indicator light remains off. Shooter speed still controls the windmill feed
 * independently of the AprilTag light.
 *
 * "Lights" are the goBILDA RGB light on Expansion Hub 2 servo port 3 (not in
 * the robot config; driven directly), the Control Hub and Expansion Hub LEDs,
 * and gamepad 2's light bar (PS4/PS5-style pads only). When off, the hubs
 * show their normal status pattern.
 *
 * Gamepad 1: left stick drive/strafe, right stick turn.
 * Gamepad 2: right trigger intake, left trigger reverse,
 *            right bumper shoot (launcher, then windmill above 35%), left bumper eject,
 *            A (hold) turbo shoot at 61.79976%.
 */
@TeleOp(name = "Jimothy TeleOp", group = "Jimothy")
public class BioBuzzStarterbotTeleopV6 extends OpMode {
    // Launcher runs at this fraction of the motor's full free speed.
    // v5 used a fixed 350 ticks/s, which is only ~12% of a goBILDA 5202.
    public static final double LAUNCHER_SPEED_FRACTION = 0.514998;
    // Turbo (gamepad 2 A held) runs the launcher at this fraction instead.
    public static final double LAUNCHER_TURBO_SPEED_FRACTION = 0.6179976;
    private static final int TURBO_RUMBLE_MS = 300;
    // Launcher readings go to the match log (tag "TeleopV6") every 0.25 s while
    // shooting, in /sdcard/FIRST/matchlogs/Match-*-Mec_BioBuzz_StarterBot_Teleop_v6.txt.
    private static final String TAG = "TeleopV6";
    private static final double LOG_PERIOD_S = 0.25;
    private static final double DRIVE_DEADZONE = 0.15;
    private static final double RED_TARGET_UPRIGHT_ROLL_TOLERANCE_DEGREES = 10.0;
    public static final double LAUNCHER_EJECT_VELOCITY = 35;
    // The windmill feeds, and the lights turn green, above this fraction of full speed.
    public static final double WINDMILL_FEED_MIN_SPEED_FRACTION = 0.35;
    // goBILDA RGB Indicator Light (same positions as Light Test / Shooter Test).
    private static final String GOBILDA_LIGHT_HUB = "Expansion Hub 2";
    private static final int GOBILDA_LIGHT_PORT = 3;
    private static final double GOBILDA_OFF = 0.0;
    private static final double GOBILDA_RED = 0.279;
    private static final double GOBILDA_GREEN = 0.500;
    private static final double GOBILDA_BLUE = 0.611;
    private static final List<Blinker.Step> SOLID_GREEN =
            Collections.singletonList(new Blinker.Step(Color.GREEN, 1, TimeUnit.SECONDS));
    private static final List<Blinker.Step> SOLID_RED =
            Collections.singletonList(new Blinker.Step(Color.RED, 1, TimeUnit.SECONDS));
    private static final List<Blinker.Step> SOLID_BLUE =
            Collections.singletonList(new Blinker.Step(Color.BLUE, 1, TimeUnit.SECONDS));

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor intake;
    private DcMotorEx launcher;
    private CRServo windmillServo;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private AprilTagProcessor aprilTags;
    private VisionPortal webcamPortal;
    private List<LynxModule> hubs;
    private ServoController gobildaLightServos;
    protected double launcherMaxVelocity;
    protected double launcherNormalVelocity;
    protected double launcherTurboVelocity;
    protected double launcherTargetVelocity;
    private double feedMinVelocity;
    protected boolean turbo;
    protected boolean lastTurbo;
    private final ElapsedTime logTimer = new ElapsedTime();

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    private double intakePower;
    private double intakeServoPower;
    private double windmillPower;

    private enum Light { OFF, RED, GREEN, BLUE }
    // null until the first loop so the lights are always set once.
    private Light currentLight = null;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        // Mechanisms are optional so a drivetrain-only robot can still use TeleOp.
        intake = hardwareMap.tryGet(DcMotor.class, "intake_motor");
        launcher = hardwareMap.tryGet(DcMotorEx.class, "shooter_motor");
        windmillServo = hardwareMap.tryGet(CRServo.class, "windmill_servo");
        leftIntakeServo = hardwareMap.tryGet(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.tryGet(CRServo.class, "right_intake_servo");
        WebcamName webcam = hardwareMap.tryGet(WebcamName.class, "Webcam 1");
        if (webcam != null) {
            aprilTags = new AprilTagProcessor.Builder()
                    .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                    .build();
            // Maximum image detail for better detection of smaller or farther tags.
            aprilTags.setDecimation(1.0f);
            webcamPortal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .setCameraResolution(new Size(1280, 720))
                    .addProcessor(aprilTags)
                    .build();
        } else {
            telemetry.addLine("Note: no 'Webcam 1'; AprilTag alignment disabled.");
        }
        hubs = hardwareMap.getAll(LynxModule.class);
        gobildaLightServos = hardwareMap.tryGet(ServoController.class, GOBILDA_LIGHT_HUB);
        if (gobildaLightServos == null) {
            telemetry.addLine("Note: no '" + GOBILDA_LIGHT_HUB + "' servo controller; goBILDA light disabled.");
        }

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotor motor : new DcMotor[] {leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (intake != null) {
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            telemetry.addLine("Note: no 'intake_motor'; intake motor disabled.");
        }
        if (launcher != null) {
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(40.0, 0.0, 0.0, 12.5));
            // Full free speed from the motor type in the robot config.
            launcherMaxVelocity = launcher.getMotorType().getMaxRPM()
                    * launcher.getMotorType().getTicksPerRev() / 60.0;
        } else {
            launcherMaxVelocity = 1.0;
            telemetry.addLine("Note: no 'shooter_motor'; shooter disabled.");
        }
        launcherNormalVelocity = normalShooterFraction() * launcherMaxVelocity;
        launcherTurboVelocity = turboShooterFraction() * launcherMaxVelocity;
        launcherTargetVelocity = launcherNormalVelocity;
        RobotLog.ii(TAG, "init max=%.0f normal=%.0f turbo=%.0f ticks/s", launcherMaxVelocity,
                launcherNormalVelocity, launcherTurboVelocity);
        feedMinVelocity = WINDMILL_FEED_MIN_SPEED_FRACTION * launcherMaxVelocity;

        if (leftIntakeServo != null) {
            leftIntakeServo.setPower(0.0);
            leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (rightIntakeServo != null) {
            rightIntakeServo.setPower(0.0);
            rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if (windmillServo != null) {
            windmillServo.setPower(0.0);
            windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        mecanumDrive(applyDriveDeadzone(-gamepad1.left_stick_y),
                applyDriveDeadzone(gamepad1.left_stick_x),
                applyDriveDeadzone(gamepad1.right_stick_x));

        if (gamepad2.right_trigger > 0) {
            intakePower = gamepad2.right_trigger;
            intakeServoPower = -gamepad2.right_trigger;
        } else if (gamepad2.left_trigger > 0) {
            intakePower = -gamepad2.left_trigger;
            intakeServoPower = intakePower;
        } else {
            intakePower = 0.0;
            intakeServoPower = 0.0;
        }

        updateTurbo();
        launch();
        double launcherVelocity = launcher == null ? 0.0 : launcher.getVelocity();
        boolean launcherFastEnough = launcherVelocity > feedMinVelocity;
        boolean shooting = gamepad2.right_bumper || turbo;
        if (shooting && logTimer.seconds() >= LOG_PERIOD_S) {
            logTimer.reset();
            RobotLog.ii(TAG, "turbo=%d rb=%d target=%.0f actual=%.0f power=%.2f", turbo ? 1 : 0,
                    gamepad2.right_bumper ? 1 : 0, launcherTargetVelocity, launcherVelocity,
                    launcher == null ? 0.0 : launcher.getPower());
        }

        setIntakePower(intakePower, intakeServoPower);

        if (shooting) {
            // Feed only above 35%; stops again whenever the launcher drops to 35% or below.
            windmillPower = launcherFastEnough ? -1.0 : 0.0;
        } else if (gamepad2.left_bumper) {
            windmillPower = 1.0;
        } else if (gamepad2.left_trigger > 0) {
            windmillPower = gamepad2.left_trigger;
        } else {
            windmillPower = 0.0;
        }
        if (windmillServo != null) windmillServo.setPower(windmillPower);

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Raw drive sticks F / S / R", "%.3f / %.3f / %.3f",
                -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.addData("Drive deadzone", "%.0f%%", DRIVE_DEADZONE * 100.0);
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)", gamepad2.left_trigger, gamepad2.right_trigger);
        telemetry.addData("Launcher", "%.0f / %.0f ticks/s (%.0f%% of max)", launcherVelocity, launcherTargetVelocity,
                100.0 * launcherVelocity / launcherMaxVelocity);
        telemetry.addData("Launcher setting", "%s: %.0f%% of %.0f ticks/s max (%s)",
                shooterSettingLabel(), 100.0 * launcherTargetVelocity / launcherMaxVelocity,
                launcherMaxVelocity, shooterSettingsDescription());
        telemetry.addData("Windmill feed", !shooting ? "-" : launcherFastEnough ? "FEEDING" : "WAITING FOR SPEED");
        telemetry.addData("Intake motor / servos / windmill", "%.2f / %.2f / %.2f", intakePower, intakeServoPower, windmillPower);
        boolean redTargetAligned = showAprilTags();
        showLight(redTargetAligned ? Light.GREEN : Light.OFF);
        telemetry.addData("Lights", "%s (requires visible upright red AprilTag)", currentLight);
        telemetry.addData("Controls", controlsDescription());
    }

    @Override
    public void stop() {
        showLight(Light.OFF);
        if (webcamPortal != null) {
            webcamPortal.close();
        }
    }

    private boolean showAprilTags() {
        if (webcamPortal == null || aprilTags == null) {
            telemetry.addData("Webcam", "NOT CONFIGURED");
            return false;
        }
        telemetry.addData("Webcam", webcamPortal.getCameraState());
        List<AprilTagDetection> detections = aprilTags.getDetections();
        telemetry.addData("AprilTags detected", detections.size());
        telemetry.addData("Detected tags", detections.isEmpty() ? "NONE" : "listed below");
        boolean redTargetVisible = false;
        boolean redTargetAligned = false;
        String closestRedTarget = "NONE";
        double closestRoll = Double.NaN;
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String name = cluster.metadata == null ? "unknown cluster" : cluster.metadata.name;
                boolean redTarget = name.equalsIgnoreCase("RED SCORING")
                        || name.equalsIgnoreCase("RED AUDIENCE");
                if (detection.ftcPose == null) {
                    telemetry.addData("Tag cluster", "%s, %d%% visible",
                            name, cluster.percentClusterFound);
                } else {
                    telemetry.addData("Tag cluster",
                            "%s, %d%% visible, range=%.1fin bearing=%+.1f yaw=%+.1f roll=%+.1f",
                            name, cluster.percentClusterFound, detection.ftcPose.range,
                            detection.ftcPose.bearing, detection.ftcPose.yaw,
                            detection.ftcPose.roll);
                    if (redTarget) {
                        redTargetVisible = true;
                        if (Double.isNaN(closestRoll)
                                || Math.abs(detection.ftcPose.roll) < Math.abs(closestRoll)) {
                            closestRoll = detection.ftcPose.roll;
                            closestRedTarget = name;
                        }
                        redTargetAligned |= Math.abs(detection.ftcPose.roll)
                                <= RED_TARGET_UPRIGHT_ROLL_TOLERANCE_DEGREES;
                    }
                }
                continue;
            }
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
            String name = single.metadata == null ? "unknown" : single.metadata.name;
            boolean redTarget = single.id >= 30 && single.id <= 37;
            if (detection.ftcPose == null) {
                telemetry.addData("Tag " + single.id, name);
            } else {
                telemetry.addData("Tag " + single.id,
                        "%s range=%.1fin bearing=%+.1f yaw=%+.1f roll=%+.1f",
                        name, detection.ftcPose.range,
                        detection.ftcPose.bearing, detection.ftcPose.yaw,
                        detection.ftcPose.roll);
                if (redTarget) {
                    redTargetVisible = true;
                    if (Double.isNaN(closestRoll)
                            || Math.abs(detection.ftcPose.roll) < Math.abs(closestRoll)) {
                        closestRoll = detection.ftcPose.roll;
                        closestRedTarget = single.id <= 33 ? "RED SCORING" : "RED AUDIENCE";
                    }
                    redTargetAligned |= Math.abs(detection.ftcPose.roll)
                            <= RED_TARGET_UPRIGHT_ROLL_TOLERANCE_DEGREES;
                }
            }
        }
        if (redTargetAligned) {
            telemetry.addData("Red orientation", "%s UPRIGHT (roll=%+.1f)",
                    closestRedTarget, closestRoll);
        } else if (redTargetVisible) {
            telemetry.addData("Red orientation", "%s visible; roll=%+.1f (need ±%.0f)",
                    closestRedTarget, closestRoll,
                    RED_TARGET_UPRIGHT_ROLL_TOLERANCE_DEGREES);
        } else {
            telemetry.addLine("Red orientation: RED SCORING / RED AUDIENCE not visible");
        }
        return redTargetAligned;
    }

    // RED/GREEN/BLUE are pushed on top of the hub's own status pattern and popped
    // for OFF, so the normal status blink shows whenever the light is off. The
    // external RGB light has PWM disabled for OFF so it goes physically dark.
    private void showLight(Light light) {
        boolean changed = light != currentLight;
        boolean hadPattern = currentLight != null && currentLight != Light.OFF;
        currentLight = light;
        if (changed) {
            for (LynxModule hub : hubs) {
                if (hadPattern) {
                    hub.popPattern();
                }
                if (light == Light.RED) {
                    hub.pushPattern(SOLID_RED);
                } else if (light == Light.GREEN) {
                    hub.pushPattern(SOLID_GREEN);
                } else if (light == Light.BLUE) {
                    hub.pushPattern(SOLID_BLUE);
                }
            }
            if (light == Light.RED) {
                gamepad2.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
            } else if (light == Light.GREEN) {
                gamepad2.setLedColor(0.0, 1.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
            } else if (light == Light.BLUE) {
                gamepad2.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
            } else {
                gamepad2.setLedColor(0.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
            }
        }
        if (gobildaLightServos != null) {
            if (light == Light.OFF) {
                gobildaLightServos.setServoPosition(GOBILDA_LIGHT_PORT, GOBILDA_OFF);
                gobildaLightServos.pwmDisable();
            } else {
                gobildaLightServos.pwmEnable();
                gobildaLightServos.setServoPosition(GOBILDA_LIGHT_PORT,
                        light == Light.RED ? GOBILDA_RED
                                : light == Light.GREEN ? GOBILDA_GREEN : GOBILDA_BLUE);
            }
        }
    }

    private void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private double applyDriveDeadzone(double input) {
        double magnitude = Math.abs(input);
        if (magnitude <= DRIVE_DEADZONE) {
            return 0.0;
        }
        return Math.copySign((magnitude - DRIVE_DEADZONE) / (1.0 - DRIVE_DEADZONE), input);
    }

    private void setIntakePower(double motorPower, double servoPower) {
        if (intake != null) intake.setPower(motorPower);
        if (leftIntakeServo != null) leftIntakeServo.setPower(servoPower);
        if (rightIntakeServo != null) rightIntakeServo.setPower(servoPower);
    }

    // Turbo while A is held. The sound and rumble fire once per press (rising
    // edge), not every loop, so holding A doesn't repeat them.
    protected void updateTurbo() {
        turbo = gamepad2.a;
        if (turbo && !lastTurbo) {
            telemetry.speak("Turbo");
            gamepad2.rumble(TURBO_RUMBLE_MS);
        }
        if (turbo != lastTurbo) {
            RobotLog.ii(TAG, "turbo %s (gamepad 2 A), target %.0f ticks/s", turbo ? "ON" : "OFF",
                    turbo ? launcherTurboVelocity : launcherNormalVelocity);
        }
        lastTurbo = turbo;
        launcherTargetVelocity = turbo ? launcherTurboVelocity : launcherNormalVelocity;
    }

    protected double normalShooterFraction() {
        return LAUNCHER_SPEED_FRACTION;
    }

    protected double turboShooterFraction() {
        return LAUNCHER_TURBO_SPEED_FRACTION;
    }

    protected String shooterSettingLabel() {
        return turbo ? "TURBO" : "normal";
    }

    protected String shooterSettingsDescription() {
        return String.format("normal %.0f%%, turbo %.0f%% on gp2 A",
                LAUNCHER_SPEED_FRACTION * 100, LAUNCHER_TURBO_SPEED_FRACTION * 100);
    }

    protected String controlsDescription() {
        return "BIOBUZZ v6 - AprilTag alignment lights";
    }

    private void launch() {
        if (launcher == null) return;
        if (gamepad2.right_bumper || turbo) {
            launcher.setVelocity(launcherTargetVelocity);
        } else if (gamepad2.left_bumper) {
            launcher.setVelocity(-LAUNCHER_EJECT_VELOCITY);
        } else {
            launcher.setVelocity(0.0);
        }
    }
}
