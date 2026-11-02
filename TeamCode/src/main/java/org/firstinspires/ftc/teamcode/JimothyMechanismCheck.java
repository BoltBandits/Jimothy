package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/** Hold-to-run diagnostic for Jimothy's non-drive mechanisms and sensors. */
@TeleOp(name = "Jimothy Mechanism Check", group = "Diagnostics")
public class JimothyMechanismCheck extends OpMode {
    private static final double INTAKE_MOTOR_POWER = 0.20;
    private static final double SERVO_POWER = 0.25;
    private static final double SHOOTER_SPEED_FRACTION = 0.20;

    private DcMotor intakeMotor;
    private DcMotor leftOdometry;
    private DcMotor rightOdometry;
    private DcMotor centerOdometry;
    private DcMotorEx shooter;
    private CRServo windmill;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private IMU imu;
    private Limelight3A limelight;
    private WebcamName webcam;
    private double shooterTestVelocity;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        windmill = hardwareMap.get(CRServo.class, "windmill_servo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        leftOdometry = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightOdometry = hardwareMap.get(DcMotor.class, "odo_right");
        centerOdometry = hardwareMap.get(DcMotor.class, "odo_center");
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40.0, 0.0, 0.0, 12.5));
        shooterTestVelocity = SHOOTER_SPEED_FRACTION
                * shooter.getMotorType().getMaxRPM()
                * shooter.getMotorType().getTicksPerRev() / 60.0;

        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        stopMechanisms();

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("REMOVE ALL GAME PIECES and raise moving parts clear");
        telemetry.addLine("Hold one control at a time; release to stop");
        telemetry.update();
    }

    @Override
    public void loop() {
        stopMechanisms();
        String test = "STOPPED";

        if (gamepad1.a) {
            intakeMotor.setPower(INTAKE_MOTOR_POWER);
            test = "INTAKE MOTOR FORWARD";
        } else if (gamepad1.b) {
            leftIntakeServo.setPower(-SERVO_POWER);
            test = "LEFT INTAKE SERVO IN";
        } else if (gamepad1.x) {
            rightIntakeServo.setPower(-SERVO_POWER);
            test = "RIGHT INTAKE SERVO IN";
        } else if (gamepad1.y) {
            leftIntakeServo.setPower(-SERVO_POWER);
            rightIntakeServo.setPower(-SERVO_POWER);
            test = "BOTH INTAKE SERVOS IN";
        } else if (gamepad1.left_bumper) {
            windmill.setPower(-SERVO_POWER);
            test = "WINDMILL FEED DIRECTION";
        } else if (gamepad1.right_bumper) {
            windmill.setPower(SERVO_POWER);
            test = "WINDMILL REVERSE DIRECTION";
        } else if (gamepad1.dpad_up) {
            shooter.setVelocity(shooterTestVelocity);
            test = "SHOOTER ONLY AT 20% - NO FEED";
        } else if (gamepad1.dpad_down) {
            intakeMotor.setPower(-INTAKE_MOTOR_POWER);
            leftIntakeServo.setPower(SERVO_POWER);
            rightIntakeServo.setPower(SERVO_POWER);
            test = "FULL INTAKE REVERSE";
        }

        telemetry.addData("Active test", test);
        telemetry.addData("Shooter target / actual", "%.0f / %.0f ticks/s",
                gamepad1.dpad_up ? shooterTestVelocity : 0.0, Math.abs(shooter.getVelocity()));
        telemetry.addData("Odometry left / right / center", "%d / %d / %d",
                leftOdometry.getCurrentPosition(), rightOdometry.getCurrentPosition(),
                centerOdometry.getCurrentPosition());
        telemetry.addData("IMU", imu != null ? "CONFIGURED" : "MISSING");
        telemetry.addData("Limelight", limelight != null ? "CONFIGURED" : "MISSING");
        telemetry.addData("Webcam 1", webcam != null ? "CONFIGURED" : "MISSING");
        telemetry.addLine("A=intake motor, B=left servo, X=right servo, Y=both servos");
        telemetry.addLine("LB=windmill feed, RB=windmill reverse");
        telemetry.addLine("D-pad Up=shooter 20%, D-pad Down=intake reverse");
        telemetry.update();
    }

    @Override
    public void stop() {
        stopMechanisms();
        limelight.stop();
    }

    private void stopMechanisms() {
        intakeMotor.setPower(0);
        shooter.setVelocity(0);
        windmill.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }
}
