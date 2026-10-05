/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Driver-controlled program for the goBILDA 2026-2027 BIOBUZZ StarterBot with
 * mecanum wheels. The hardware names match Jimothy's starter_bot configuration.
 */
@TeleOp(name = "Mec BioBuzz StarterBot Teleop v5", group = "StarterBot")
public class BioBuzzStarterbotTeleopMecanum extends OpMode {

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    public static final int LAUNCHER_TARGET_VELOCITY = 350;
    // Reverse/eject at 10% of the normal shooting velocity.
    public static final int LAUNCHER_EJECT_VELOCITY = 35;
    public static final int LAUNCHER_READY_TOLERANCE = 15;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    private double intakePower;
    private double intakeServoPower;
    private double windmillPower;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        launcher = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        windmillServo = hardwareMap.get(CRServo.class, "windmill_servo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // This robot's drivetrain is mounted opposite the conventional mecanum
        // translation signs. Invert forward and strafe while preserving rotation.
        mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        // Right trigger intakes; left trigger reverses the intake to eject.
        if (gamepad2.right_trigger > 0) {
            intakePower = gamepad2.right_trigger;
            intakeServoPower = -gamepad2.right_trigger;
        } else if (gamepad2.left_trigger > 0) {
            intakePower = -gamepad2.left_trigger;
            intakeServoPower = intakePower;
        } else {
            intakePower = 0;
            intakeServoPower = 0;
        }
        launch();
        double launcherVelocity = launcher.getVelocity();
        boolean launcherReady = Math.abs(
                launcherVelocity - LAUNCHER_TARGET_VELOCITY) <= LAUNCHER_READY_TOLERANCE;

        setIntakePower(intakePower, intakeServoPower);

        if (gamepad2.right_bumper) {
            // Feed inward immediately while the shooter spins up and runs.
            windmillPower = -1;
        } else if (gamepad2.left_bumper) {
            windmillPower = 1;
        } else if (gamepad2.left_trigger > 0) {
            windmillPower = gamepad2.left_trigger;
        } else {
            windmillPower = 0;
        }

        windmillServo.setPower(windmillPower);

        telemetry.addData(
                "Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData(
                "Triggers", "left (%.2f), right (%.2f)",
                gamepad2.left_trigger, gamepad2.right_trigger);
        telemetry.addData("Launcher", "%.0f / %d ticks/s (%s)",
                launcherVelocity, LAUNCHER_TARGET_VELOCITY,
                launcherReady ? "READY" : "SPINNING UP");
        telemetry.addData("Intake motor / servos / windmill", "%.2f / %.2f / %.2f",
                intakePower, intakeServoPower, windmillPower);
        telemetry.addData("Controls", "BIOBUZZ v5 - intake ports 0/1 + motor");
    }

    private void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

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

    /** Run the intake motor and both intake servos together. */
    private void setIntakePower(double motorPower, double servoPower) {
        intakePower = motorPower;
        intakeServoPower = servoPower;
        intake.setPower(motorPower);
        leftIntakeServo.setPower(servoPower);
        rightIntakeServo.setPower(servoPower);
    }

    private void launch() {
        if (gamepad2.right_bumper) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad2.left_bumper) {
            launcher.setVelocity(-LAUNCHER_EJECT_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

    }
}
