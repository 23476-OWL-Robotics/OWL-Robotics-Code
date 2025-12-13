package org.firstinspires.ftc.teamcode.Util.Vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;

import androidx.annotation.ColorInt;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Util.ArtifactType;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class ArtifactProcessor implements VisionProcessor {

    ArrayList<VisionArtifact> userArtifacts = new ArrayList<>();
    ImageRegion roiImg;
    Rect roi;
    Mat roiMat;
    Mat roiMat_userColorSpace;
    Mat maskGreen = new Mat();
    Mat maskPurple = new Mat();
    int frameWidth;
    int frameHeight;
    int minBlobSize;
    int maxBlobSize;


    // ToDo: Remove DrawFrame stuff once Working
    final boolean drawContours = true;
    private final Paint boundingRectPaint;
    private final Paint roiPaint;
    private final Paint circleFitPaint;
    private final Paint contourPaint;
    final @ColorInt int circleFitColor = Color.rgb(0, 255, 0);
    final @ColorInt int boundingBoxColor = Color.rgb(255, 120, 31);
    final @ColorInt int roiColor = Color.rgb(255, 255, 255);
    final @ColorInt int contourColor = Color.rgb(3, 227, 252);

    public static class Builder {

        private ImageRegion imageRegion;
        private int minBlobSize;
        private int maxBlobSize;

        private boolean sizeSet;

        public Builder setRoi(ImageRegion roi) {
            this.imageRegion = roi;
            return this;
        }

        public Builder setPreferredBlobSize(int min, int max) {
            minBlobSize = min;
            maxBlobSize = max;
            sizeSet = true;
            return this;
        }

        public ArtifactProcessor build() {
            if (imageRegion == null) {
                throw new IllegalArgumentException("You Must Set a Range Of Interest (ROI)!");
            }

            if (!sizeSet) {
                throw new IllegalArgumentException("You Must Set a Preferred Blob Size!");
            }

            return new ArtifactProcessor(this);
        }
    }

    private ArtifactProcessor(Builder builder) {
        this.roiImg = builder.imageRegion;
        this.minBlobSize = builder.minBlobSize;
        this.maxBlobSize = builder.maxBlobSize;

        // ToDo: Remove DrawFrame stuff once Working
        roiPaint = new Paint();
        roiPaint.setAntiAlias(true);
        roiPaint.setStrokeCap(Paint.Cap.BUTT);
        roiPaint.setColor(roiColor);

        // ToDo: Remove DrawFrame stuff once Working
        contourPaint = new Paint();
        contourPaint.setStyle(Paint.Style.STROKE);
        contourPaint.setColor(contourColor);

        // ToDo: Remove DrawFrame stuff once Working
        boundingRectPaint = new Paint();
        boundingRectPaint.setAntiAlias(true);
        boundingRectPaint.setStrokeCap(Paint.Cap.BUTT);
        boundingRectPaint.setColor(boundingBoxColor);

        // ToDo: Remove DrawFrame stuff once Working
        circleFitPaint = new Paint();
        circleFitPaint.setAntiAlias(true);
        circleFitPaint.setStrokeCap(Paint.Cap.BUTT);
        circleFitPaint.setStyle(Paint.Style.STROKE);
        circleFitPaint.setColor(circleFitColor);
    }


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.frameWidth = width;
        this.frameHeight = height;

        this.roi = roiImg.asOpenCvRect(frameWidth, frameHeight);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (roiMat == null)
        {
            roiMat = frame.submat(roi);
            roiMat_userColorSpace = roiMat.clone();
        }

        Imgproc.cvtColor(roiMat, roiMat_userColorSpace, Imgproc.COLOR_BGR2YCrCb);

        Core.inRange(roiMat_userColorSpace, ColorRange.ARTIFACT_GREEN.min, ColorRange.ARTIFACT_GREEN.max, maskGreen);
        Core.inRange(roiMat_userColorSpace, ColorRange.ARTIFACT_PURPLE.min, ColorRange.ARTIFACT_PURPLE.max, maskPurple);

        ArrayList<VisionArtifact> artifacts = new ArrayList<>();
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(maskGreen, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
            Core.add(contour, new Scalar(roi.x, roi.y), contour);
            artifacts.add(new VisionArtifact(contour, ArtifactType.GREEN));
        }
        contours.clear();

        Imgproc.findContours(maskPurple, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
            Core.add(contour, new Scalar(roi.x, roi.y), contour);
            artifacts.add(new VisionArtifact(contour, ArtifactType.PURPLE));
        }

        hierarchy.release();

        ArrayList<VisionArtifact> toRemove = new ArrayList<>();
        for (VisionArtifact a : artifacts) {
            if (a.getContourArea() > maxBlobSize || a.getContourArea() < minBlobSize) {
                toRemove.add(a);
            }
        }
        artifacts.removeAll(toRemove);

        userArtifacts = new ArrayList<>(artifacts);

        return artifacts;
    }

    // ToDo: Remove DrawFrame stuff once Working
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        ArrayList<VisionArtifact> artifacts = (ArrayList<VisionArtifact>) userContext;

        contourPaint.setStrokeWidth(scaleCanvasDensity * 4);
        if (boundingRectPaint != null)
        {
            boundingRectPaint.setStrokeWidth(scaleCanvasDensity * 10);
        }
        if (circleFitPaint != null)
        {
            circleFitPaint.setStrokeWidth(scaleCanvasDensity * 10);
        }
        roiPaint.setStrokeWidth(scaleCanvasDensity * 10);

        android.graphics.Rect gfxRect = makeGraphicsRect(roi, scaleBmpPxToCanvasPx);

        for (VisionArtifact artifact : artifacts)
        {
            if (drawContours)
            {
                Path path = new Path();

                Point[] contourPts = artifact.getContourPoints();

                path.moveTo((float) (contourPts[0].x) * scaleBmpPxToCanvasPx, (float)(contourPts[0].y) * scaleBmpPxToCanvasPx);
                for (int i = 1; i < contourPts.length; i++)
                {
                    path.lineTo((float) (contourPts[i].x) * scaleBmpPxToCanvasPx, (float) (contourPts[i].y) * scaleBmpPxToCanvasPx);
                }
                path.close();

                canvas.drawPath(path, contourPaint);
            }

            /*
             * Draws a rotated rect by drawing each of the 4 lines individually
             */
            if (boundingRectPaint != null)
            {
                Point[] rotRectPts = new Point[4];
                artifact.getBoxFit().points(rotRectPts);

                for (int i = 0; i < 4; ++i)
                {
                    canvas.drawLine(
                            (float) (rotRectPts[i].x) * scaleBmpPxToCanvasPx, (float) (rotRectPts[i].y) * scaleBmpPxToCanvasPx,
                            (float) (rotRectPts[(i + 1) % 4].x) * scaleBmpPxToCanvasPx, (float) (rotRectPts[(i + 1) % 4].y) * scaleBmpPxToCanvasPx,
                            boundingRectPaint
                    );
                }
            }

            if (circleFitPaint != null)
            {
                Circle circle = artifact.getCircle();
                canvas.drawCircle(
                        circle.getX() * scaleBmpPxToCanvasPx, circle.getY() * scaleBmpPxToCanvasPx,
                        circle.getRadius() * scaleBmpPxToCanvasPx, circleFitPaint
                );
            }
        }

        canvas.drawLine(gfxRect.left, gfxRect.top, gfxRect.right, gfxRect.top, roiPaint);
        canvas.drawLine(gfxRect.right, gfxRect.top, gfxRect.right, gfxRect.bottom, roiPaint);
        canvas.drawLine(gfxRect.right, gfxRect.bottom, gfxRect.left, gfxRect.bottom, roiPaint);
        canvas.drawLine(gfxRect.left, gfxRect.bottom, gfxRect.left, gfxRect.top, roiPaint);
    }

    // ToDo: Remove DrawFrame stuff once Working
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx)
    {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    public ArrayList<VisionArtifact> getArtifacts() {
        return userArtifacts;
    }
}
