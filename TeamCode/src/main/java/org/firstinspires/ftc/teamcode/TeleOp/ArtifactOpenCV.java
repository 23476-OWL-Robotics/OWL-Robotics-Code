package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;
import android.util.Size;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Util.Vision.VisionArtifact;
import org.firstinspires.ftc.teamcode.Util.Vision.ArtifactProcessor;
import org.firstinspires.ftc.teamcode.Util.Vision.ImageRegion;
import org.firstinspires.ftc.teamcode.Util.Vision.VisionUtil;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;

@TeleOp
@Disabled
public class ArtifactOpenCV extends OpMode {

    VisionPortal portal;
    ArtifactProcessor artifactProcessor;
    ArrayList<VisionArtifact> artifacts;
    Timer loopTime = new Timer();

    @Override
    public void init() {
        artifactProcessor = new ArtifactProcessor.Builder()
                .setRoi(ImageRegion.entireFrame())
                .setPreferredBlobSize(1000, 10000)
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(artifactProcessor)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Artifact Camera"))
                .build();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        loopTime.resetTimer();

        artifacts = artifactProcessor.getArtifacts();

        VisionUtil.sortByArea(SortOrder.DESCENDING, artifacts);

        telemetry.addLine("Area | Circularity | Center | Type");
        for (VisionArtifact a : artifacts) {
            telemetry.addLine(
                    String.format("%3d %.4f (%3d, %3d) %S",
                            a.getContourArea(),
                            a.getCircularity(),
                            (int)a.getCircle().getCenter().x, (int)a.getCircle().getCenter().y,
                            a.getArtifactType())
            );
        }

        telemetry.addLine();
        telemetry.addData("Loop Time", loopTime.getElapsedTime());
        telemetry.update();
    }
}
