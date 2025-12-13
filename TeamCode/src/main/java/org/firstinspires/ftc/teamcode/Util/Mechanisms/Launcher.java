package org.firstinspires.ftc.teamcode.Util.Mechanisms;

import android.util.Size;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Util.PIDFController.Coefficients;
import org.firstinspires.ftc.teamcode.Util.PIDFController.ControllerStates;
import org.firstinspires.ftc.teamcode.Util.PIDFController.VelocityController;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.Utilities;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point3;

import java.util.ArrayList;

public class Launcher {

    OpMode opMode;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    VelocityController vController;

    VisionPortal tagPortal;
    AprilTagProcessor tagProcessor;

    DcMotorEx launcherMotor;

    Servo rotationServo;
    Servo leftAngleServo;
    Servo rightAngleServo;

    Point3 goalPose;

    // Gravity in Inches Per Second
    // DO NOT CHANGE
    final double g = 386.2205;

    // Pi (Self Explanatory)
    // DO NOT CHANGE
    final double pi = 3.1415926535;

    // Wheel Diameter in Inches
    // DO NOT CHANGE
    final double wheelDiameter = 2.8346;
    final double wheelCircumference = pi * wheelDiameter;

    final double launchIncPerDeg = 0.03;
    final double rotationIncPerDeg = 0.0026666667;

    final double launcherHeight = 10.25;

    final double RotationServoZero = 0.5;
    final double RotationServoOffset = 0.5;

    final double LaunchServoZero = 0.07;
    final double LaunchServoOffset = 0.07;

    double velocity = 0;
    double launchAngle = 0;
    double rotationAngle = 0;
    double rotationOffset = 0;
    double rotationAngleThreshold = 35;

    double cameraBearing = 0;

    double launchPosition = 0;
    double rotationPosition = 0;

    boolean legalPose = false;
    boolean launcherEnabled = false;
    boolean reverseMotor = false;

    public Launcher(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    public void init() {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");

        rotationServo = hardwareMap.get(Servo.class, "launcherRotationServo");
        leftAngleServo = hardwareMap.get(Servo.class, "launcherLeftAngleServo");
        rightAngleServo = hardwareMap.get(Servo.class, "launcherRightAngleServo");

        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rotationServo.setDirection(Servo.Direction.FORWARD);
        leftAngleServo.setDirection(Servo.Direction.FORWARD);
        rightAngleServo.setDirection(Servo.Direction.REVERSE);

        Utilities.Set_PWM_Range(leftAngleServo, new PwmControl.PwmRange(500, 2500));
        Utilities.Set_PWM_Range(rightAngleServo, new PwmControl.PwmRange(500, 2500));

        rotationServo.setPosition(RotationServoZero);
        leftAngleServo.setPosition(LaunchServoZero);
        rightAngleServo.setPosition(LaunchServoZero);

        vController = new VelocityController.Builder()
                .setCoefficients(new Coefficients.VelocityCoefficients.LauncherMotorCoefficients())
                .setEndState(ControllerStates.RUN_CONTROLLER)
                .setEndVelocityError(20)
                .build();

        vController.setState(ControllerStates.STOP_CONTROLLER);

        tagProcessor = new AprilTagProcessor.Builder()
                .build();

        tagPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Tag Camera"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tagProcessor)
                .build();
        tagPortal.stopLiveView();
    }

    public void loop(Pose robotPose) {

        if (launcherEnabled) {
            RunCalculations(robotPose);

            launchPosition = ((70-launchAngle) * launchIncPerDeg) + LaunchServoOffset;
            rotationPosition = (rotationAngle * rotationIncPerDeg) + RotationServoOffset;

            int setV = (int) (2 * (velocity * wheelCircumference));
            vController.setTargetRPM(setV-100);
            vController.setState(ControllerStates.RUN_CONTROLLER);
            vController.runController(launcherMotor.getVelocity());
            launcherMotor.setPower(vController.getOut());
        } else {
            launchPosition = LaunchServoZero;
            rotationPosition = RotationServoZero;

            if (reverseMotor) {
                launcherMotor.setPower(-0.08);
            } else {
                launcherMotor.setPower(0);
            }
            vController.setState(ControllerStates.STOP_CONTROLLER);
        }

        leftAngleServo.setPosition(launchPosition);
        rightAngleServo.setPosition(launchPosition);
        rotationServo.setPosition(rotationPosition);
    }

    public void Telemetry() {
        telemetry.addLine("----Calculations----");
        telemetry.addData("Legal Pose?", legalPose);
        telemetry.addData("Launch Angle", launchAngle);
        telemetry.addData("Rotation Angle", rotationAngle);
        telemetry.addData("Camera Bearing", cameraBearing);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("RPM", 2*(velocity * wheelCircumference));
        telemetry.addLine("----Servo Positions----");
        telemetry.addData("Launch Servo Position", launchPosition);
        telemetry.addData("Rotation Servo Position", rotationPosition);
        telemetry.addLine("----Launch Motor----");
        telemetry.addData("Target Speed", vController.getTargetRPM());
        telemetry.addData("Motor Speed", vController.getCurrentRPM());
        telemetry.addData("Motor Power", vController.getOut());
        telemetry.addData("Controller State", vController.getState());
    }

    /*
    public void startMotor() {
        int setV = (int) (2 * (velocity * wheelCircumference));
        vController.setTargetRPM(setV);
        vController.setState(ControllerStates.RUN_CONTROLLER);
    }
    public void stopMotor() {
        vController.setTargetVelocity(0);
        vController.setState(ControllerStates.STOP_CONTROLLER);
    }

     */

    public void setLauncherEnabled(boolean enabled) {
        this.launcherEnabled = enabled;
    }

    public void reverseMotor() {
        reverseMotor = true;
    }
    public void stopReverse() {
        reverseMotor = false;
    }

    public void setTargetGoal(Point3 goalPose) {
        this.goalPose = goalPose;
    }

    public AprilTagProcessor getTagProcessor() {
        return tagProcessor;
    }

    public Point3 poseToPoint3(Pose pose, double z) {
        double x = pose.getX();
        double y = pose.getY();

        return new Point3(x, y, z);
    }

    private void RunCalculations(Pose robotPose) {
        if (!IsLegalLaunchingPosition(robotPose)) {
            launchAngle = 70;
            rotationAngle = 0;
            rotationOffset = 0;
            velocity = 0;
            return;
        }

        CalculateRotationAngle(robotPose);
        CalculateLaunchAngle(poseToPoint3(robotPose, launcherHeight));
    }

    private boolean IsLegalLaunchingPosition(Pose robotPose) {
        boolean value = false;

        // Check if the Robot is within the small scoring zone;
        if (35 <= robotPose.getX() && robotPose.getX() < 109 && 0 <= robotPose.getY() && robotPose.getY() <= 31) {

            if (robotPose.getY() <= robotPose.getX() +35 && robotPose.getY() <= -robotPose.getX() + 109){

                if (goalPose == Utilities.RedGoalPose) {
                    rotationOffset = -11;
                } else if (goalPose == Utilities.BlueGoalPose){
                    rotationOffset = 20;
                }

                value = true;
            }
        }

        // Check if the Robot is withing the Large Scoring Zone
        if (0 <= robotPose.getX() && robotPose.getX() <= 144 && 65 <= robotPose.getY() && robotPose.getY() <= 144) {

            if (robotPose.getY() >= robotPose.getX() - 15 && robotPose.getY() >= -robotPose.getX() + 129) {
                value = true;
            }
        }

        legalPose = value;
        return value;
    }

    /**
        This function finds the launch angle from the position of the launcher and
        the position of the goal.
        Both are in relation to the origin of the field which is at the Bottom Left of the field.
     **/
    private void CalculateLaunchAngle(Point3 launchPose) {

        // Needed Velocity
        double v;

        // Needed Angle
        double a;

        // x is the distance from base of the robot to the base of the Goal.
        // y is the Height from the shooter to the height of the goal.
        double x, y;

        // Use pythagorean theorem to find the distance from base of the robot to the base of the Goal.
        x = Math.sqrt(
                Math.pow(Math.abs(launchPose.x - goalPose.x), 2) +
                        Math.pow(Math.abs(launchPose.y - goalPose.y), 2));

        // Find Gy. Absolute value isn't needed but helps ensure that we do not divide by a negative number later.
        y = Math.abs(goalPose.z - launchPose.z);

        if (x < 30) {
            return;
        }


        // Find the velocity (Best Equation I could come up with
        v = Math.pow((250 * x), 0.5) + 73;

        // Set the velocity
        velocity = v;

        /*
            Finding the launch angle is actually rather simple once you have x, y, v, and g

            Its like a quadratic equation:

            v squared +- sqrt (v power 4 - g( gx squared + 2yv squared )
            ------------------------------------------------------------
                                       gx

            Then you take the arc-tangent of that to find the angle
        */
        double rad = Math.pow(v, 4) - (g * ((g * Math.pow(x, 2) ) + (2 * y * Math.pow(v, 2))));

        a = Math.atan(
            (Math.pow(v, 2) + Math.sqrt(rad)) /
            (g * x)
            );

        if (Double.isNaN(a)) {
            a = Math.toRadians(70);
        }

        // Once we have calculated the angle, we must convert it from radians to degrees
        launchAngle = Math.toDegrees(a);
    }

    private void CalculateRotationAngle(Pose robotPose) {

        // Rotation Angle
        double a;

        // Robots Distance from the Goal
        double x, y;

        // First, we will find the current needed rotation angle from the robots current heading

        // Find the x and y values
        x = robotPose.getX() - goalPose.x;
        y = robotPose.getY() - goalPose.y;

        double theta = Math.toDegrees(Math.atan(y / x));
        if (theta < 0) theta = Math.abs(theta) + 90;

        a = Math.abs(Math.toDegrees(robotPose.getHeading()) - 180) + theta;

        if (a > 180) a = a-360;

        if (a < rotationAngleThreshold && a > -rotationAngleThreshold) {
            ArrayList<AprilTagDetection> detections = filterByID(tagProcessor.getDetections());

            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);
                a += tag.ftcPose.bearing;
                cameraBearing = tag.ftcPose.bearing;
                telemetry.addData("Tag Bearing", tag.ftcPose.bearing);
            }
        } else {
            a = 0;
        }

        a += rotationOffset;


        // Set the rotation angle
        rotationAngle = a;
    }

    private ArrayList<AprilTagDetection> filterByID(ArrayList<AprilTagDetection> detections) {

        int TagID = 0;

        if (goalPose == Utilities.RedGoalPose) {
            TagID = 24;
        } else if (goalPose == Utilities.BlueGoalPose) {
            TagID = 25;
        }

        ArrayList<AprilTagDetection> toRemove = new ArrayList<>();

        for (AprilTagDetection d : detections) {

            if (d.id != TagID) {
                toRemove.add(d);
            }
        }

        detections.removeAll(toRemove);
        return detections;
    }
}
