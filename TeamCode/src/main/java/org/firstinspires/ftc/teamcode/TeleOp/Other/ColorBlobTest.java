package org.firstinspires.ftc.teamcode.TeleOp.Other;

// Android Imports
import android.annotation.SuppressLint;
import android.os.Environment;

// FTC Imports
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SortOrder;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorSpace;

// OpenCV imports
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

//Java Imports
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class ColorBlobTest extends LinearOpMode
{
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        // Edge Detection Threshold values
        int Threshold1 = 110;
        int Threshold2 = Threshold1 * 3;

        // Blob size values
        int maxBlobSize = 15000;
        int minBlobSize = 2000;

        // Vision Portal for Camera
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // Blobs and contours for Blob Detection
        ArrayList<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        // File names
        String srcFileName = Environment.getExternalStorageDirectory().getPath() + "/VisionPortal-Frame.png";
        // Color Range for blocks
        ColorRange colorRange = ColorRange.YELLOW;
        String colorRangeString = "Yellow";

        // OpenCV Mats
        // Mat is basically a class with two data parts: the matrix header (containing information such as the size of the matrix, the method used for storing,
        // at which address is the matrix stored, and so on) and a pointer to the matrix containing the pixel values (taking any dimensionality depending
        // on the method chosen for storing) . The matrix header size is constant, however the size of the matrix itself may vary from image to image and
        // usually is larger by orders of magnitude
        Mat blur = new Mat();
        Mat mask = new Mat();
        Mat hierarchy = new Mat();
        Mat result = new Mat();
        Mat edges = new Mat();
        Mat roiMat;
        Mat roiMat_userColorSpace;
        Mat srcFrame;

        // ROI (Range of Interest)
        Rect roi;
        ImageRegion roiImg = ImageRegion.asUnityCenterCoordinates(-0.9, 0.9, 0.9, -0.9);

        waitForStart();
        if (opModeIsActive()) {
            //portal.stopStreaming();
            while (opModeIsActive()) {

                // Change colorRange
                if (gamepad1.b) {
                    colorRange = ColorRange.BLUE;
                    colorRangeString = "Blue";
                } else if (gamepad1.x) {
                    colorRange = ColorRange.YELLOW;
                    colorRangeString = "Yellow";
                } else if (gamepad1.y) {
                    colorRange = ColorRange.RED;
                    colorRangeString = "Red";
                }

                if (gamepad1.a) {
                    // Clear the blobs and contours ArrayList
                    blobs.clear();
                    contours.clear();

                    // Save the next frame from the camera
                    // This will take some time so we need a sleep function to make sure it fully saves.
                    // If it does not save, the robot will crash (bad)
                    portal.saveNextFrameRaw("Frame");
                    try {
                        TimeUnit.MILLISECONDS.sleep(1000);
                    } catch (InterruptedException e) {
                        //Nothing
                    }

                    // Open the frame with OpenCV
                    srcFrame = Imgcodecs.imread(srcFileName);

                    // Blur the image for Edge Detection. This will make the edges more visible.
                    Imgproc.blur(srcFrame, blur, new Size(3, 3));
                    // Canny Edge Detection
                    Imgproc.Canny(blur, edges, Threshold1, Threshold2, 3, true);
                    // Make the edge lines thicker to separate blocks that are very close together.
                    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
                    Imgproc.dilate(edges, edges, kernel);

                    // Add the edges to the original image
                    Core.subtract(srcFrame, edges, result);

                    // Change the roi for blob detection
                    roi = roiImg.asOpenCvRect(1280, 720);
                    roiMat = result.submat(roi);
                    roiMat_userColorSpace = roiMat.clone();

                    // For blob detection, the first step is to convert the image to BGR (Blue Green Red)
                    // Do not use RGB as opencv uses BRG
                    Imgproc.cvtColor(roiMat, roiMat_userColorSpace, Imgproc.COLOR_BGR2YCrCb);
                    // Then the image is converted to black and white.
                    // The inRange function only makes pixels with an acceptable color range white.
                    Core.inRange(roiMat_userColorSpace, colorRange.min, colorRange.max, mask);
                    // findContours is the blob detection
                    // It finds all blobs of white pixels and adds them to the array contours
                    Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                    hierarchy.release();

                    // Adds all of the found contours to the blobs array
                    // This will allow us to get any necessary information about the blobs
                    for (MatOfPoint contour : contours)
                    {
                        Core.add(contour, new Scalar(roi.x, roi.y), contour);
                        blobs.add(new BlobImpl(contour));
                    }

                    // We will remove blobs that are either too large or too small
                    // This stops single pixels from being counted as blocks.
                    ArrayList<ColorBlobLocatorProcessor.Blob> toRemove = new ArrayList<>();
                    for(ColorBlobLocatorProcessor.Blob b : blobs)
                    {
                        if (b.getContourArea() > maxBlobSize || b.getContourArea() < minBlobSize)
                        {
                            toRemove.add(b);
                        }
                    }
                    blobs.removeAll(toRemove);

                    // Finally we sort the blobs by density.
                    // The blob with the highest density is the one we will want to grab
                    sortByDensity(SortOrder.DESCENDING, blobs);
                }

                telemetry.addData("Color ", colorRangeString);
                telemetry.addData("Camera Status", portal.getCameraState());
                telemetry.addLine();
                telemetry.addLine(" Area Density Aspect  Center");
                for(ColorBlobLocatorProcessor.Blob b : blobs)
                {
                    double angle;

                    RotatedRect boxFit = b.getBoxFit();
                    Point[] boxPoints = new Point[4];
                    boxFit.points(boxPoints);

                    if (boxFit.angle == 0 && (boxPoints[3].x - boxPoints[1].x) > 400) {
                        angle = 90;
                    } else if (boxPoints[1].x < boxPoints[3].x) {
                        angle = (90 - boxFit.angle) + 90;
                    } else {
                        angle = boxFit.angle;
                    }

                    telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d %.4f)",
                            b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y, angle)
                    );
                    telemetry.addLine(String.format("(%d,%3d)  (%3d,%3d)  (%3d,%3d)  (%3d,%3d)",
                            (int) boxPoints[0].x, (int) boxPoints[0].y,
                            (int) boxPoints[1].x, (int) boxPoints[1].y,
                            (int) boxPoints[2].x, (int) boxPoints[2].y,
                            (int) boxPoints[3].x, (int) boxPoints[3].y)
                    );
                }
                telemetry.update();
            }
        }
    }

    static class ColorRange {
        final ColorSpace colorSpace;
        final Scalar min;
        final Scalar max;

        // -----------------------------------------------------------------------------
        // DEFAULT OPTIONS
        // -----------------------------------------------------------------------------

        public static ColorRange BLUE = new ColorRange(
                ColorSpace.YCrCb,
                new Scalar(0,   0, 145),
                new Scalar(255, 127, 255)
        );

        public static final ColorRange RED = new ColorRange(
                ColorSpace.YCrCb,
                new Scalar( 32, 190,  0),
                new Scalar(255, 255, 132)
        );

        public static final ColorRange YELLOW = new ColorRange(
                ColorSpace.YCrCb,
                new Scalar( 82, 128,   0),
                new Scalar(255, 190, 115)
        );

        public static final ColorRange GREEN = new ColorRange(
                ColorSpace.YCrCb,
                new Scalar( 32,   0,   0),
                new Scalar(255, 120, 133)
        );

        // -----------------------------------------------------------------------------
        // ROLL YOUR OWN
        // -----------------------------------------------------------------------------

        public ColorRange(ColorSpace colorSpace, Scalar min, Scalar max)
        {
            this.colorSpace = colorSpace;
            this.min = min;
            this.max = max;
        }
    }

    class BlobImpl extends ColorBlobLocatorProcessor.Blob {
        private MatOfPoint contour;
        private Point[] contourPts;
        private int area = -1;
        private double density = -1;
        private double aspectRatio = -1;
        private RotatedRect rect;

        BlobImpl(MatOfPoint contour)
        {
            this.contour = contour;
        }

        @Override
        public MatOfPoint getContour()
        {
            return contour;
        }

        @Override
        public Point[] getContourPoints()
        {
            if (contourPts == null)
            {
                contourPts = contour.toArray();
            }

            return contourPts;
        }

        @Override
        public int getContourArea()
        {
            if (area < 0)
            {
                area = Math.max(1, (int) Imgproc.contourArea(contour));  //  Fix zero area issue
            }

            return area;
        }

        @Override
        public double getDensity()
        {
            Point[] contourPts = getContourPoints();

            if (density < 0)
            {
                // Compute the convex hull of the contour
                MatOfInt hullMatOfInt = new MatOfInt();
                Imgproc.convexHull(contour, hullMatOfInt);

                // The convex hull calculation tells us the INDEX of the points which
                // which were passed in earlier which form the convex hull. That's all
                // well and good, but now we need filter out that original list to find
                // the actual POINTS which form the convex hull
                Point[] hullPoints = new Point[hullMatOfInt.rows()];
                List<Integer> hullContourIdxList = hullMatOfInt.toList();

                for (int i = 0; i < hullContourIdxList.size(); i++)
                {
                    hullPoints[i] = contourPts[hullContourIdxList.get(i)];
                }

                double hullArea = Math.max(1.0,Imgproc.contourArea(new MatOfPoint(hullPoints)));  //  Fix zero area issue

                density = getContourArea() / hullArea;
            }
            return density;
        }

        @Override
        public double getAspectRatio()
        {
            if (aspectRatio < 0)
            {
                RotatedRect r = getBoxFit();

                double longSize  = Math.max(1, Math.max(r.size.width, r.size.height));
                double shortSize = Math.max(1, Math.min(r.size.width, r.size.height));

                aspectRatio = longSize / shortSize;
            }

            return aspectRatio;
        }

        @Override
        public RotatedRect getBoxFit()
        {
            if (rect == null)
            {
                rect = Imgproc.minAreaRect(new MatOfPoint2f(getContourPoints()));
            }
            return rect;
        }
    }

    static class ImageRegion {
        final boolean imageCoords;
        final double left, top, right, bottom;

        private ImageRegion(boolean imageCoords, double left, double top, double right, double bottom ) {
            this.left = left;
            this.top = top;
            this.right = right;
            this.bottom = bottom;
            this.imageCoords = imageCoords;
        }

        public static ImageRegion asImageCoordinates(int left, int top, int right, int bottom ) {
            return new ImageRegion(true, left, top, right, bottom);
        }

        public static ImageRegion asUnityCenterCoordinates(double left, double top, double right, double bottom) {
            return new ImageRegion(false, left, top, right, bottom);
        }

        public static ImageRegion entireFrame() {
            return ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1);
        }

        protected Rect asOpenCvRect(int imageWidth, int imageHeight) {
            Rect rect = new Rect();

            if (imageCoords) {
                rect.x = (int) left;
                rect.y = (int) top;
                rect.width = (int) (right - left);
                rect.height = (int) (bottom - top);
            }
            else {
                rect.x = (int) Range.scale(left, -1, 1, 0, imageWidth);
                rect.y = (int) ( imageHeight - Range.scale(top, -1, 1, 0, imageHeight));
                rect.width = (int) Range.scale(right - left, 0, 2, 0, imageWidth);
                rect.height = (int) Range.scale(top - bottom, 0, 2, 0, imageHeight);
            }

            // Adjust the window position to ensure it stays on the screen.  push it back into the screen area.
            // We could just crop it instead, but then it may completely miss the screen.
            rect.x = Math.max(rect.x, 0);
            rect.x = Math.min(rect.x, imageWidth - rect.width);
            rect.y = Math.max(rect.y, 0);
            rect.y = Math.min(rect.y, imageHeight - rect.height);

            return rect;
        }
    }
    public static void sortByDensity(SortOrder sortOrder, List<ColorBlobLocatorProcessor.Blob> blobs) {
        blobs.sort(new Comparator<ColorBlobLocatorProcessor.Blob>()
        {
            public int compare(ColorBlobLocatorProcessor.Blob c1, ColorBlobLocatorProcessor.Blob c2)
            {
                int tmp = (int)Math.signum(c2.getDensity() - c1.getDensity());

                if (sortOrder == SortOrder.ASCENDING)
                {
                    tmp = -tmp;
                }

                return tmp;
            }
        });
    }
}
