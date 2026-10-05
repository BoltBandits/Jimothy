package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Individually verifies the Jimothy intake and windmill hardware mapping. */
@TeleOp(name = "BioBuzz Intake Hardware Test", group = "Diagnostics")
public class BioBuzzIntakeHardwareTest extends OpMode {
    private DcMotorEx intakeMotor;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        windmillServo = hardwareMap.get(CRServo.class, "windmill_servo");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        stopAll();
    }

    @Override
    public void loop() {
        stopAll();
        String active = "none";

        if (gamepad2.dpad_up) {
            intakeMotor.setPower(0.35);
            active = "UP: intake motor (Expansion Hub 2 port 0)";
        } else if (gamepad2.dpad_left) {
            leftIntakeServo.setPower(0.35);
            active = "LEFT: left intake servo (Control Hub port 0)";
        } else if (gamepad2.dpad_right) {
            rightIntakeServo.setPower(0.35);
            active = "RIGHT: right intake servo (Control Hub port 1)";
        } else if (gamepad2.dpad_down) {
            windmillServo.setPower(0.35);
            active = "DOWN: windmill servo (Control Hub port 2)";
        }

        telemetry.addLine("BIOBUZZ v5 individual hardware test");
        telemetry.addLine("Gamepad 2 D-pad: UP=motor, LEFT=left intake");
        telemetry.addLine("RIGHT=right intake, DOWN=windmill");
        telemetry.addData("Active", active);
        telemetry.addData("Intake motor command", "%.2f", intakeMotor.getPower());
        telemetry.addData("Intake motor velocity", "%.0f ticks/s", intakeMotor.getVelocity());
        telemetry.addData("Intake motor current", "%.2f A",
                intakeMotor.getCurrent(CurrentUnit.AMPS));
    }

    @Override
    public void stop() {
        stopAll();
    }

    private void stopAll() {
        intakeMotor.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
    }
}
