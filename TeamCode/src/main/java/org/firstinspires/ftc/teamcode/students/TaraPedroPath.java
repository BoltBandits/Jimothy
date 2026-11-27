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
import com.pedropathing.paths.interpolator.Interpolator;

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

        final Pose start = poseFactory.of(54.9219, 7.7305, 270);
        final Pose path1 = poseFactory.of(7.4857, 7.9695, 270);
        final Pose path1Control1 = poseFactory.of(30.924, 30.6199, 0);
        final Pose point2Start = poseFactory.of(7.4857, 7.9695, 0);
        final Pose point2 = poseFactory.of(47.1092, 127.6301, 90);
        final Pose point2Control1 = poseFactory.of(39.5059, 27.2134, 0);
        final Pose point2Control2 = poseFactory.of(15.2936, 115.089, 0);
        final Pose point3 = poseFactory.of(12.0397, 115.8362, 90);
        final Pose point3Control1 = poseFactory.of(31.4669, 113.796, 0);



        Path path = Paths.path(
                Paths.curve(start, path1Control1, path1).constant(path1),
                Paths.curve(point2Start, point2Control1, point2Control2, point2).linear(point2Start, point2),
                Paths.curve(point2, point3Control1, point3).constant(point3)
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

