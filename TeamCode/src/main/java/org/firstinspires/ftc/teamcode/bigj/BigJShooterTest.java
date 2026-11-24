package org.firstinspires.ftc.teamcode.bigj;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.RobotHookups;

@TeleOp(name = "Big J Shooter Test", group = "Big J Tests")
public class BigJShooterTest extends OpMode {
    private static final double TEST_VELOCITY_TICKS_PER_SECOND = 500;
    private DcMotorEx shooter;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, RobotHookups.Names.SHOOTER);
        shooter.setPower(0);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Press START, then hold A to run the shooter.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double target = gamepad1.a ? TEST_VELOCITY_TICKS_PER_SECOND : 0;
        shooter.setVelocity(target);
        telemetry.addData("Command (ticks/s)", "%.0f", target);
        telemetry.addData("Measured (ticks/s)", "%.0f", shooter.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.setPower(0);
    }
}
