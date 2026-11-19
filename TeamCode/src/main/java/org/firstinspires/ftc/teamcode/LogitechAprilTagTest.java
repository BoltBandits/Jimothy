package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/** Standalone, no-motion test for the Logitech webcam and BioBuzz AprilTags. */
@TeleOp(name = "Logitech AprilTag Test", group = "Diagnostics")
public class LogitechAprilTagTest extends LinearOpMode {
    private static final String TAG = "LogitechTagTest";

    @Override
    public void runOpMode() {
        WebcamName webcam = hardwareMap.tryGet(WebcamName.class, "Webcam 1");
        if (webcam == null) {
            telemetry.addLine("ERROR: Webcam 1 is not configured or connected.");
            telemetry.update();
            RobotLog.ee(TAG, "Webcam 1 is not configured or connected");
            waitForStart();
            return;
        }

        AprilTagProcessor aprilTags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .build();
        aprilTags.setDecimation(1.0f);

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTags)
                .build();

        telemetry.addLine("Logitech camera initialized at 640x480 MJPEG.");
        telemetry.addLine("Press Start, then hold a BioBuzz AprilTag in view.");
        telemetry.update();
        RobotLog.ii(TAG, "Initialized Webcam 1 at 640x480 MJPEG");

        waitForStart();
        if (isStopRequested()) {
            portal.close();
            return;
        }

        int previousCount = -1;
        try {
            while (opModeIsActive()) {
                List<AprilTagDetection> detections = aprilTags.getDetections();
                telemetry.addData("Camera", portal.getCameraState());
                telemetry.addData("Detections", detections.size());
                telemetry.addLine(detections.isEmpty()
                        ? "NO APRILTAG DETECTED" : "APRILTAG DETECTED");

                if (detections.size() != previousCount) {
                    RobotLog.ii(TAG, "Camera=%s detections=%d",
                            portal.getCameraState(), detections.size());
                    previousCount = detections.size();
                }

                for (AprilTagDetection detection : detections) {
                    if (detection instanceof AprilTagClusterDetection) {
                        AprilTagClusterDetection cluster =
                                (AprilTagClusterDetection) detection;
                        String name = cluster.metadata == null
                                ? "unknown cluster" : cluster.metadata.name;
                        telemetry.addData("Cluster", "%s (%d%% visible)",
                                name, cluster.percentClusterFound);
                        RobotLog.ii(TAG, "Cluster name=%s visible=%d%%",
                                name, cluster.percentClusterFound);
                    } else if (detection instanceof AprilTagSingleDetection) {
                        AprilTagSingleDetection single =
                                (AprilTagSingleDetection) detection;
                        String name = single.metadata == null
                                ? "unknown" : single.metadata.name;
                        telemetry.addData("Tag " + single.id, name);
                        RobotLog.ii(TAG, "Single id=%d name=%s", single.id, name);
                    } else {
                        telemetry.addLine("Unknown AprilTag detection type");
                    }

                    if (detection.ftcPose != null) {
                        telemetry.addData("Pose",
                                "range %.1f in, bearing %+.1f, yaw %+.1f, roll %+.1f",
                                detection.ftcPose.range, detection.ftcPose.bearing,
                                detection.ftcPose.yaw, detection.ftcPose.roll);
                    }
                }
                telemetry.update();
                sleep(50);
            }
        } finally {
            portal.close();
            RobotLog.ii(TAG, "Test stopped; webcam closed");
        }
    }
}
