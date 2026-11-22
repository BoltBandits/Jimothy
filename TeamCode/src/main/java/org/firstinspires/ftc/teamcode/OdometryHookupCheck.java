package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Odometry Hookup Check", group = "Jimothy Tests")
public class OdometryHookupCheck extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHookups hookups = RobotHookups.JIMOTHY_LEGACY;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_left) hookups = RobotHookups.JIMOTHY_LEGACY;
            else if (gamepad1.dpad_right) hookups = RobotHookups.BIG_J_CHART;
            telemetry.addData("Selected profile", hookups);
            telemetry.addLine("D-pad Left = Jimothy, Right = Big J");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        DcMotor left = hardwareMap.get(DcMotor.class, hookups.leftOdometry);
        DcMotor right = hardwareMap.get(DcMotor.class, hookups.rightOdometry);
        DcMotor strafe = hardwareMap.get(DcMotor.class, hookups.strafeOdometry);
        int leftStart = left.getCurrentPosition();
        int rightStart = right.getCurrentPosition();
        int strafeStart = strafe.getCurrentPosition();
        while (opModeIsActive()) {
            telemetry.addData("Profile", hookups);
            telemetry.addData("Left (" + hookups.leftOdometry + ")", left.getCurrentPosition() - leftStart);
            telemetry.addData("Right (" + hookups.rightOdometry + ")", right.getCurrentPosition() - rightStart);
            telemetry.addData("Strafe (" + hookups.strafeOdometry + ")", strafe.getCurrentPosition() - strafeStart);
            telemetry.update();
            idle();
        }
    }
}
