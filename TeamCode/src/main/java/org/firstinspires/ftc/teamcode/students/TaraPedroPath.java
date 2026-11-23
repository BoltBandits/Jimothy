package org.firstinspires.ftc.teamcode.students;


import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.students.Constants;
import static com.pedropathing.api.Paths.line;

@Autonomous(group = "PedroTests")
public class TaraPedroPath extends LinearOpMode {
    double distance = 48;
    Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0,0,Math.toRadians(90)));

        double distance = 48;
        boolean forward = true;

        final PoseFactory poseFactory = PoseFactory.degrees();

        final Pose start = poseFactory.of(56, 8, 90);
        final Pose path1 = poseFactory.of(10.7425, 13.7643, 180);
        final Pose path1Control1 = poseFactory.of(25.6744, 24.5519, 0);
        final Pose point2 = poseFactory.of(45.4091, 133.458, -176.5759);
        final Pose point2Control1 = poseFactory.of(69.1177, 0, 0);
        final Pose point2Control2 = poseFactory.of(11.8977, 131.4529, 0);

        Path path = Paths.path(
                Paths.curve(start, path1Control1, path1).linear(start, path1),
                Paths.curve(path1, point2Control1, point2Control2, point2).reverseTangent()
        );



        Thread.sleep(1000);
        waitForStart();
        follower.setPose(start);
        follower.update();
        follower.follow(path);

        while (opModeIsActive()) {
            follower.update();


        }
    }
}

