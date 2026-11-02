package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Low-power, hold-to-run diagnostic for verifying drive motor mapping and direction. */
@TeleOp(name = "Jimothy Drive Motor Check", group = "Diagnostics")
public class JimothyDriveMotorCheck extends OpMode {
    private static final double TEST_POWER = 0.15;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        configureMotor(frontLeft);
        configureMotor(frontRight);
        configureMotor(backLeft);
        configureMotor(backRight);

        telemetry.addLine("RAISE THE ROBOT so all wheels are off the floor");
        telemetry.addLine("Hold A=FL, B=FR, X=BL, Y=BR");
        telemetry.addLine("Hold D-pad Up=all four forward");
        telemetry.update();
    }

    @Override
    public void loop() {
        double fl = 0;
        double fr = 0;
        double bl = 0;
        double br = 0;
        String activeTest = "STOPPED";

        if (gamepad1.dpad_up) {
            fl = fr = bl = br = TEST_POWER;
            activeTest = "ALL FORWARD";
        } else if (gamepad1.a) {
            fl = TEST_POWER;
            activeTest = "FRONT LEFT";
        } else if (gamepad1.b) {
            fr = TEST_POWER;
            activeTest = "FRONT RIGHT";
        } else if (gamepad1.x) {
            bl = TEST_POWER;
            activeTest = "BACK LEFT";
        } else if (gamepad1.y) {
            br = TEST_POWER;
            activeTest = "BACK RIGHT";
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        telemetry.addData("Test", "%s at %.0f%%", activeTest, TEST_POWER * 100);
        telemetry.addData("Command FL / FR", "%.2f / %.2f", fl, fr);
        telemetry.addData("Command BL / BR", "%.2f / %.2f", bl, br);
        telemetry.addData("Encoder FL / FR", "%d / %d",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        telemetry.addData("Encoder BL / BR", "%d / %d",
                backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        telemetry.addLine("Release every control to stop immediately");
        telemetry.update();
    }

    @Override
    public void stop() {
        stopMotors();
    }

    private void configureMotor(DcMotor motor) {
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
