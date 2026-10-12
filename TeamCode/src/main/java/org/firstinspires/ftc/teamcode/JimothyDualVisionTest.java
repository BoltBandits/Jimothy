package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "Jimothy Dual Vision Test", group = "Diagnostics")
public class JimothyDualVisionTest extends LinearOpMode {
    private static final int LIMELIGHT_PIPELINE = 0;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        WebcamName rearWebcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        AprilTagLibrary tagLibrary = AprilTagGameDatabase.getBioBuzzTagLibrary();
        AprilTagProcessor aprilTags = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .build();
        VisionPortal webcamPortal = new VisionPortal.Builder()
                .setCamera(rearWebcam)
                .addProcessor(aprilTags)
                .build();

        telemetry.setMsTransmissionInterval(50);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        telemetry.addLine("Forward Limelight: pipeline 0 neural detector");
        telemetry.addLine("Rear webcam: AprilTag detector");
        telemetry.addLine("Press START to monitor both cameras");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                showLimelight(limelight);
                showAprilTags(webcamPortal, aprilTags);
                telemetry.update();
                idle();
            }
        } finally {
            limelight.stop();
            webcamPortal.close();
        }
    }

    private void showLimelight(Limelight3A limelight) {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Limelight", "pipeline %d (%s), %.0f FPS",
                status.getPipelineIndex(), status.getPipelineType(), status.getFps());

        LLResult result = limelight.getLatestResult();
        if (result == null) {
            telemetry.addLine("  No Limelight result yet");
            return;
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        int recognized = 0;
        for (LLResultTypes.DetectorResult detection : detections) {
            String label = detection.getClassName();
            if (!isGamePiece(label)) continue;
            recognized++;
            telemetry.addData("  LL " + recognized,
                    "%s conf=%.0f%% tx=%+.1f ty=%+.1f area=%.2f",
                    label, detection.getConfidence() * 100.0,
                    detection.getTargetXDegrees(), detection.getTargetYDegrees(),
                    detection.getTargetArea());
        }
        if (recognized == 0) {
            telemetry.addLine("  No pollen/red_nectar/blue_nector detected");
            for (LLResultTypes.DetectorResult detection : detections) {
                telemetry.addData("  Raw LL class", "%s (%.0f%%)",
                        detection.getClassName(), detection.getConfidence() * 100.0);
            }
        }
    }

    private void showAprilTags(VisionPortal portal, AprilTagProcessor processor) {
        telemetry.addData("Rear webcam", "%s", portal.getCameraState());
        List<AprilTagDetection> detections = processor.getDetections();
        telemetry.addData("AprilTags", detections.size());
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String clusterName = cluster.metadata == null
                        ? "unknown cluster" : cluster.metadata.name;
                String shortName = cluster.metadata == null
                        ? "unknown" : cluster.metadata.shortName;
                if (detection.ftcPose == null) {
                    telemetry.addData("  Cluster " + shortName,
                            "%s, %d%% visible", clusterName, cluster.percentClusterFound);
                } else {
                    telemetry.addData("  Cluster " + shortName,
                            "%s, %d%% visible, range=%.1fin bearing=%+.1f yaw=%+.1f",
                            clusterName, cluster.percentClusterFound,
                            detection.ftcPose.range, detection.ftcPose.bearing,
                            detection.ftcPose.yaw);
                }
                continue;
            }
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
            String name = single.metadata == null
                    ? "unknown" : single.metadata.name;
            if (detection.ftcPose == null) {
                telemetry.addData("  Tag " + single.id, "%s", name);
            } else {
                telemetry.addData("  Tag " + single.id,
                        "%s range=%.1fin bearing=%+.1f yaw=%+.1f",
                        name, detection.ftcPose.range,
                        detection.ftcPose.bearing, detection.ftcPose.yaw);
            }
        }
    }

    private boolean isGamePiece(String label) {
        if (label == null) return false;
        String normalized = label.toLowerCase(Locale.US).trim();
        return normalized.equals("pollen")
                || normalized.equals("red_nectar")
                || normalized.equals("blue_nector")
                || normalized.equals("blue_nectar");
    }
}
