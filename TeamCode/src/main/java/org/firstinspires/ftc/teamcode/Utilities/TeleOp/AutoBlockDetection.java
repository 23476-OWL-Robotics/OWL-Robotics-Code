package org.firstinspires.ftc.teamcode.Utilities.TeleOp;

import android.os.Environment;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.IntakeControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.PIDF_Controller;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class AutoBlockDetection {

    WebcamName webcam;
    VisionPortal portal;

    HardwareMap hardwareMap;
    DcMotorEx intakeMotor;
    Servo armPivot;
    Servo armRotate;
    Servo armGrab;

    double xInchesPerPixel = 0.023828125;
    double yInchesPerPixel = 0.033333333;
    double servoMultiplier = 0.00333;

    public double angle;
    public double robotPosX;
    public double robotPosY;

    public double blockPosX;
    public double blockPosY;
    public double blockAngle;

    int cameraWidth = 640;
    int cameraHeight = 480;

    int Threshold1 = 110;
    int Threshold2 = Threshold1 * 3;

    int maxBlobSize = 15000;
    int minBlobSize = 2000;

    // Blobs and contours for Blob Detection
    ArrayList<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();
    ArrayList<MatOfPoint> contours = new ArrayList<>();
    // File names
    String srcFileName = Environment.getExternalStorageDirectory().getPath() + "/VisionPortal-Frame.png";
    String resultFileName = Environment.getExternalStorageDirectory().getPath() + "/gray.png";
    // Color Range for blocks
    ColorRange colorRange;

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

    public AutoBlockDetection(Builder builder) {
        this.hardwareMap = builder.hardwareMap;
        webcam = this.hardwareMap.get(WebcamName.class, "Webcam 1");

        this.colorRange = builder.colorRange;

        this.intakeMotor = builder.intakeMotor;

        this.armPivot = builder.armPivot;
        this.armRotate = builder.armRotate;
        this.armGrab = builder.armGrab;
    }
    public static class Builder {
        HardwareMap hardwareMap;
        ColorRange colorRange;
        DcMotorEx intakeMotor;
        Servo armPivot;
        Servo armRotate;
        Servo armGrab;

        public Builder setHardWareMap(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            return this;
        }
        public Builder setColorRange(ColorRange colorRange) {
            this.colorRange = colorRange;
            return this;
        }
        public Builder setMotor(DcMotorEx motor) {
            this.intakeMotor = motor;
            return this;
        }
        public Builder setPivotServo(Servo servo) {
            this.armPivot = servo;
            return this;
        }
        public Builder setRotateServo(Servo servo) {
            this.armRotate = servo;
            return this;
        }
        public Builder setGrabServo(Servo servo) {
            this.armGrab = servo;
            return this;
        }

        public AutoBlockDetection build() {
            return new AutoBlockDetection(this);
        }
    }

    public void init() {
        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    public void findSample() {
        // Clear the blobs and contours ArrayList
        blobs.clear();
        contours.clear();

        // Save the next frame from the camera
        // This will take some time so we need a sleep function to make sure it fully saves.
        // If it does not save, the robot will crash (bad)
        portal.saveNextFrameRaw("Frame");
        try {
            TimeUnit.MILLISECONDS.sleep(500);
        } catch (InterruptedException e) {
            //Nothing
        }

        // Open the frame with OpenCV
        srcFrame = Imgcodecs.imread(srcFileName);

        // Blur the image for Edge Detection. This will make the edges more visible.
        Imgproc.blur(srcFrame, blur, new org.opencv.core.Size(3, 3));
        // Canny Edge Detection
        Imgproc.Canny(blur, edges, Threshold1, Threshold2, 3, true);
        // Make the edge lines thicker to separate blocks that are very close together.
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(2, 2));
        Imgproc.dilate(edges, edges, kernel);

        Imgcodecs.imwrite(resultFileName, edges);
        edges = Imgcodecs.imread(resultFileName);

        // Add the edges to the original image
        Core.subtract(srcFrame, edges, result);

        // Change the roi for blob detection
        roi = roiImg.asOpenCvRect(640, 480);
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

        // Get the first blob from the array
        ColorBlobLocatorProcessor.Blob b = blobs.get(0);
        // Create a rotated rectangle for the blobs box fit
        RotatedRect boxFit = b.getBoxFit();

        // Corner Points for the rectangle
        Point[] boxPoints = new Point[4];
        boxFit.points(boxPoints);

        // Set the block x and y positions relative to the robot
        blockPosX = (int) (boxFit.center.x - (roi.width / 2));
        blockPosY = roi.height - boxFit.center.y;

        // Calculate blockAngel
        if (boxFit.angle == 0 && (boxPoints[3].x - boxPoints[1].x) > 400) {
            blockAngle = 90;
        } else if (boxPoints[1].x < boxPoints[3].x) {
            blockAngle = (90 - boxFit.angle) + 90;
        } else {
            blockAngle = boxFit.angle;
        }

        // Set values for the robot to move to
        robotPosX = blockPosX * xInchesPerPixel;
        robotPosY = (blockPosY * yInchesPerPixel) - 3;
        angle = Math.abs(blockAngle - 180) * servoMultiplier;
    }

    public void grabSample(MecanumDrive drive) {
        drive.updatePoseEstimate();
        Action xMove = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d((drive.pose.position.x + robotPosX), drive.pose.position.y))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                xMove,
                                yMove(),
                                lowerPivot(),
                                rotateClaw()
                        ),
                        gramSample(),
                        raisePivot(),
                        rotateClaw(),
                        yReset()
                )
        );
    }

    public class LowerPivot implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            armPivot.setPosition(0.07);
            return false;
        }
    }
    public Action lowerPivot() {
        return new LowerPivot();
    }

    public class RaisePivot implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            armPivot.setPosition(0.76);
            angle = 0;
            return false;
        }
    }
    public Action raisePivot() {
        return new RaisePivot();
    }

    public class RotateClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            armRotate.setPosition(angle);
            return false;
        }
    }
    public Action rotateClaw() {
        return new RotateClaw();
    }

    public class GrabSample implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            armGrab.setPosition(0.89);
            return false;
        }
    }
    public Action gramSample() {
        return new GrabSample();
    }

    public class YMove implements Action {
        private boolean initialized = false;
        PIDF_Controller controller;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                IntakeControllerParams controllerParams = new IntakeControllerParams();
                controller = new PIDF_Controller.Builder()
                        .setControllerMotor(intakeMotor)
                        .setControllerParams(controllerParams.params)
                        .setMaxSpeed(0.8)
                        .setStopOnTargetReached(true)
                        .setEndPositionError(5)
                        .build();

                controller.extendTo(robotPosY);
                initialized = true;
            }

            if (controller.running) {
                controller.loopController();
                return true;
            } else {
                return false;
            }
        }
    }
    public Action yMove() {
        return new YMove();
    }

    public class YReset implements Action {
        private boolean initialized = false;
        PIDF_Controller controller;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                IntakeControllerParams controllerParams = new IntakeControllerParams();
                controller = new PIDF_Controller.Builder()
                        .setControllerMotor(intakeMotor)
                        .setControllerParams(controllerParams.params)
                        .setMaxSpeed(0.8)
                        .setStopOnTargetReached(true)
                        .setEndPositionError(5)
                        .build();

                controller.retractTo(0);
                initialized = true;
            }

            if (controller.running) {
                controller.loopController();
                return true;
            } else {
                return false;
            }
        }
    }
    public Action yReset() {
        return new YMove();
    }

    public static class ColorRange {
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

    static class BlobImpl extends ColorBlobLocatorProcessor.Blob {
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
