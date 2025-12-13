package org.firstinspires.ftc.teamcode.Util.Vision;

import org.firstinspires.ftc.teamcode.Util.ArtifactType;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class VisionArtifact {

    MatOfPoint contour;
    ArtifactType type;

    public VisionArtifact(MatOfPoint contour, ArtifactType type) {
        this.contour = contour;
        this.type = type;
    }

    public MatOfPoint getContour() {
        return contour;
    }

    public Point[] getContourPoints() {
        return contour.toArray();
    }

    public MatOfPoint2f getContourAsFloat() {
        return new MatOfPoint2f(getContourPoints());
    }

    public int getContourArea() {

        return Math.max(1, (int) Imgproc.contourArea(contour));
    }

    public double getDensity() {
        Point[] contourPts = getContourPoints();

        // Compute the convex hull of the contour
        MatOfInt hullMatOfInt = new MatOfInt();
        Imgproc.convexHull(contour, hullMatOfInt);

        // The convex hull calculation tells us the INDEX of the points which
        // which were passed in eariler which form the convex hull. That's all
        // well and good, but now we need filter out that original list to find
        // the actual POINTS which form the convex hull
        Point[] hullPoints = new Point[hullMatOfInt.rows()];
        List<Integer> hullContourIdxList = hullMatOfInt.toList();

        for (int i = 0; i < hullContourIdxList.size(); i++)
        {
            hullPoints[i] = contourPts[hullContourIdxList.get(i)];
        }

        double hullArea = Math.max(1.0,Imgproc.contourArea(new MatOfPoint(hullPoints)));  //  Fix zero area issue

        return getContourArea() / hullArea;
    }

    public double getAspectRatio() {
        RotatedRect r = getBoxFit();

        double longSize  = Math.max(1, Math.max(r.size.width, r.size.height));
        double shortSize = Math.max(1, Math.min(r.size.width, r.size.height));

        return longSize / shortSize;
    }

    public RotatedRect getBoxFit() {
        return Imgproc.minAreaRect(getContourAsFloat());
    }

    public double getArcLength() {
        return Imgproc.arcLength(getContourAsFloat(), true);
    }

    public double getCircularity() {
        return 4 * Math.PI * (getContourArea() / Math.pow(getArcLength(), 2));
    }

    public Circle getCircle() {
        Point center = new Point();
        float[] radius = new float[1];
        Imgproc.minEnclosingCircle(getContourAsFloat(), center, radius);

        return new Circle(center, radius[0]);
    }

    public ArtifactType getArtifactType() {
        return type;
    }
}
