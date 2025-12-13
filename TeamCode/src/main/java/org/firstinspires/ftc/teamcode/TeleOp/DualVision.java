package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;
import android.util.Size;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Util.Vision.VisionArtifact;
import org.firstinspires.ftc.teamcode.Util.Vision.ArtifactProcessor;
import org.firstinspires.ftc.teamcode.Util.Vision.ImageRegion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
@Disabled
public class DualVision extends OpMode {

    VisionPortal artifactPortal;
    VisionPortal aprilTagPortal;
    ArtifactProcessor artifactProcessor;
    AprilTagProcessor aprilTagProcessor;

    Timer loopTimer;

    @Override
    public void init() {

        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);
        int portal1ViewId = viewIds[0];
        int portal2ViewId = viewIds[1];

        artifactProcessor = new ArtifactProcessor.Builder()
                .setPreferredBlobSize(1000, 10000)
                .setRoi(ImageRegion.entireFrame())
                .build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .build();

        artifactPortal = new VisionPortal.Builder()
                .addProcessor(artifactProcessor)
                .setCameraResolution(new Size(1920, 1080))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam Artifact"))
                .setLiveViewContainerId(portal1ViewId)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        aprilTagPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(1280, 720))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam April Tag"))
                .setLiveViewContainerId(portal2ViewId)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        loopTimer = new Timer();
    }

    @Override
    public void loop() {
        loopTimer.resetTimer();

        ArtifactTelemetry();
        AprilTagTelemetry();

        telemetry.addLine();
        telemetry.addData("Loop Time", loopTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    public void ArtifactTelemetry() {
        //VisionUtil.sortByArea(SortOrder.DESCENDING, artifacts);

        telemetry.addData("Artifacts Detected", artifactProcessor.getArtifacts().size());

        telemetry.addLine("Area | Circularity | Center | Type");
        for (VisionArtifact a : artifactProcessor.getArtifacts()) {
            telemetry.addLine(
                    String.format("%3d %.4f (%3d, %3d) %S",
                            a.getContourArea(),
                            a.getCircularity(),
                            (int)a.getCircle().getCenter().x, (int)a.getCircle().getCenter().y,
                            a.getArtifactType())
            );
        }
    }

    @SuppressLint("DefaultLocale")
    public void AprilTagTelemetry() {
        telemetry.addData("# AprilTags Detected", aprilTagProcessor.getDetections().size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
