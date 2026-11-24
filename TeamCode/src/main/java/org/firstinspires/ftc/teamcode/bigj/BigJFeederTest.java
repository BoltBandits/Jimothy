package org.firstinspires.ftc.teamcode.bigj;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.RobotHookups;

@TeleOp(name = "Big J Feeder Test", group = "Big J Tests")
public class BigJFeederTest extends OpMode {
    private static final double TEST_POWER = 0.25;
    private DcMotor feeder;

    @Override
    public void init() {
        feeder = hardwareMap.get(DcMotor.class, RobotHookups.Names.FEEDER);
        feeder.setPower(0);
        // On Big J, this channel's encoder belongs to the left odometry pod.
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Press START, then hold A to run the feeder.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double power = gamepad1.a ? TEST_POWER : 0;
        feeder.setPower(power);
        telemetry.addData("Commanded power", "%.2f", power);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (feeder != null) feeder.setPower(0);
    }
}
