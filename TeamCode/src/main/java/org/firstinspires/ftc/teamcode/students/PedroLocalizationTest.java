package org.firstinspires.ftc.teamcode.students;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Push-by-hand test for validating Pedro's three-wheel localizer. */
@TeleOp(name = "Pedro Localization Test", group = "PedroTests")
public class PedroLocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        ThreeWheelLocalizer localizer =
                new ThreeWheelLocalizer(hardwareMap, Constants.localizerConfig);

        telemetry.addLine("PUSH-BY-HAND TEST - drivetrain is not powered");
        telemetry.addLine("Place the robot at the test origin, then press Start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        localizer.setPose(Pose.zero());

        while (opModeIsActive()) {
            localizer.update();
            Pose pose = localizer.pose();

            telemetry.addData("X forward", "%+.2f in", pose.x());
            telemetry.addData("Y left", "%+.2f in", pose.y());
            telemetry.addData("Heading CCW", "%+.1f deg",
                    Math.toDegrees(pose.heading()));
            telemetry.addLine();
            telemetry.addLine("Expected directions:");
            telemetry.addLine("Forward push -> X increases");
            telemetry.addLine("Left strafe -> Y increases");
            telemetry.addLine("CCW turn -> heading increases");
            telemetry.update();

            idle();
        }
    }
}
