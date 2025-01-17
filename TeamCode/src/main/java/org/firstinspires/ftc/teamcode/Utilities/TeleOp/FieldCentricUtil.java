package org.firstinspires.ftc.teamcode.Utilities.TeleOp;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.ArmControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.IntakeControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.LeftAssentControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.RightAssentControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.PIDF_Controller;
import org.firstinspires.ftc.teamcode.Utilities.TeleOp.FileWriter.FileReadWriter;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

public class FieldCentricUtil extends LinearOpMode {

    @Override
    public void runOpMode() {

    }

    // Create All Motors
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;
    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx rightAssentMotor;
    DcMotorEx leftAssentMotor;
    DcMotorEx armMotor;
    DcMotorEx intakeMotor;

    // Create All Servos
    Servo intakePivot;
    Servo armPivot;
    Servo sampleServo;
    Servo plowServo;

    // Create All CR Servos
    CRServo left;
    CRServo right;

    // Create sensor and imu
    ColorSensor sensor;
    IMU imu;

    // Create revBlinkinLedDriver for Led Strip Lights
    RevBlinkinLedDriver revBlinkinLedDriver;

    // Drive Motor Powers
    double front_left_power;
    double front_right_power;
    double back_left_power;
    double back_right_power;

    // Servo Powers
    double intake_wheel_power;

    // Other Variables
    double rotX;
    double rotY;
    double speedModifier;
    boolean isUp;
    boolean transfer;
    boolean doingIntake;
    boolean touchSet;
    boolean whichTF;
    double heading;
    boolean plowUp;
    boolean isIntakeSlideAuto;

    // Values for Touchpad
    double TFOneRefX;
    double TFOneRefY;
    double TFTwoRefX;
    double TFTwoRefY;
    double TFOneX;
    double TFOneY;
    double TFTwoX;
    double TFTwoY;

    boolean oldTouch;
    boolean oldInSlide;

    double armEncoderPosition;
    double intakeEncoderPosition;
    double leftAssentEncoderPosition;
    double rightAssentEncoderPosition;

    double robotPoseX;
    double robotPoseY;
    double robotHeading;

    FileReadWriter fileReadWriter;
    MecanumDrive drive;
    ElapsedTime time;

    Pose2d startPose;

    // Hardware Maps
    public void hardwareMaps() {
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");

        rightAssentMotor = hardwareMap.get(DcMotorEx.class, "rightAssentMotor");
        leftAssentMotor = hardwareMap.get(DcMotorEx.class, "leftAssentMotor");

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        armPivot = hardwareMap.get(Servo.class, "sampleServo");
        sampleServo = hardwareMap.get(Servo.class, "specimenClaw");
        plowServo = hardwareMap.get(Servo.class, "plowServo");

        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");

        sensor = hardwareMap.get(ColorSensor.class, "blockDet");
        revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "rev");

        imu = hardwareMap.get(IMU.class, "imu");
    }

    public void Initialize() {
        try {
            fileReadWriter = new FileReadWriter();
            fileReadWriter.readFile();
        } catch (IOException e) {
            // Nothing
        }

        time = new ElapsedTime();

        armEncoderPosition = fileReadWriter.read[1];
        intakeEncoderPosition = fileReadWriter.read[2];
        leftAssentEncoderPosition = fileReadWriter.read[3];
        rightAssentEncoderPosition = fileReadWriter.read[4];
        robotPoseX = fileReadWriter.read[5];
        robotPoseY = fileReadWriter.read[6];
        robotHeading = fileReadWriter.read[7];

        startPose = new Pose2d(robotPoseX, robotPoseY, Math.toRadians(robotHeading));
        drive = new MecanumDrive(hardwareMap, startPose);

        frontRightMotor = drive.rightFront;
        frontLeftMotor = drive.leftFront;
        backRightMotor = drive.rightBack;
        backLeftMotor = drive.leftBack;

        imu = drive.lazyImu.get();

        rightAssentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftAssentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightAssentMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(CRServo.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftAssentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightAssentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();

        intakePivot.setPosition(0.5);
        armPivot.setPosition(0.8);
        sampleServo.setPosition(0.78);
        plowServo.setPosition(0.8);
        plowUp = true;
        isUp = true;
        oldTouch = false;
        isIntakeSlideAuto = false;

        revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }

    // Servo Positions on opModeInInit()
    public void initializeServos() {
        intakePivot.setPosition(0.5);
        armPivot.setPosition(0.8);
        sampleServo.setPosition(0.71);
        isUp = true;
    }

    public void initializeLights() {
        revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }

    public void FieldCentric(double yStick, double xStick, double turnStick, boolean slowMode) {
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 90;
        xStick = -xStick;
        rotX = xStick * Math.cos(-heading / 180 * Math.PI) - yStick * Math.sin(-heading / 180 * Math.PI);
        rotY = xStick * Math.sin(-heading / 180 * Math.PI) + yStick * Math.cos(-heading / 180 * Math.PI);

        if (gamepad2.touchpad_finger_1 && gamepad2.touchpad_finger_2) {
            if (!touchSet) {
                touchSet = true;
                if (gamepad2.touchpad_finger_1_x > gamepad2.touchpad_finger_2_x) {
                    TFOneRefX = gamepad2.touchpad_finger_1_x;
                    TFOneRefY = -gamepad2.touchpad_finger_1_y;
                    TFTwoRefX = gamepad2.touchpad_finger_2_x;
                    TFTwoRefY = gamepad2.touchpad_finger_2_y;
                    whichTF = true;
                } else {
                    TFTwoRefX = gamepad2.touchpad_finger_1_x;
                    TFTwoRefY = gamepad2.touchpad_finger_1_y;
                    TFOneRefX = gamepad2.touchpad_finger_2_x;
                    TFOneRefY = -gamepad2.touchpad_finger_2_y;
                    whichTF = false;
                }
            } else {
                if (whichTF) {
                    TFOneX = gamepad2.touchpad_finger_1_x;
                    TFOneY = -gamepad2.touchpad_finger_1_y;
                    TFTwoX = gamepad2.touchpad_finger_2_x;
                    TFTwoY = gamepad2.touchpad_finger_2_y;
                } else {
                    TFTwoX = gamepad2.touchpad_finger_1_x;
                    TFTwoY = gamepad2.touchpad_finger_1_y;
                    TFOneX = gamepad2.touchpad_finger_2_x;
                    TFOneY = -gamepad2.touchpad_finger_2_y;
                }
                front_left_power = ((TFOneY - TFOneRefY)
                        + (TFOneX - TFOneRefX)
                        + (TFTwoX - TFTwoRefX)) * 0.4;
                front_right_power = ((TFOneY - TFOneRefY)
                        - (TFOneX - TFOneRefX)
                        - (TFTwoX - TFTwoRefX)) * 0.4;
                back_left_power = ((TFOneY - TFOneRefY)
                        - (TFOneX - TFOneRefX)
                        + (TFTwoX - TFTwoRefX)) * 0.4;
                back_right_power = ((TFOneY - TFOneRefY)
                        + (TFOneX - TFOneRefX)
                        - (TFTwoX - TFTwoRefX)) * 0.5;
            }
        } else {
            front_left_power = rotY + rotX + turnStick;
            front_right_power = (rotY - rotX) - turnStick;
            back_left_power = (rotY - rotX) + turnStick;
            back_right_power = (rotY + rotX) - turnStick;
        }

        if (slowMode) {
            speedModifier = 0.3;
        } else speedModifier = 1;
        backLeftMotor.setPower(back_left_power * speedModifier);
        backRightMotor.setPower(back_right_power * speedModifier);
        frontLeftMotor.setPower(front_left_power * speedModifier);
        frontRightMotor.setPower(front_right_power * speedModifier);
    }

    // Sets motor power sets
    public void power_sets() {
        servo_power_sets();
    }

    // Sets wrist servo to positions set by slide control
    public void sample_control(boolean output_up, boolean output_down) {
        if (output_up) {
            OutputSample outputSample = new OutputSample();
            Thread outputThread = new Thread(outputSample);

            outputThread.start();
        } else if (output_down) {
            armPivot.setPosition(0.1);
        }
    }

    // Opens and closes specimen claw
    public void specimen_control(boolean grab_specimen, boolean release_specimen) {
        if (grab_specimen) {
            sampleServo.setPosition(0.78);
        } else if (release_specimen) {
            sampleServo.setPosition(0.65);
        }
    }

    public void plowControl(boolean runPlow) {
        if (runPlow && intakeEncoderPosition > 950 && oldTouch == false) {
            PlowSamples plowSamples = new PlowSamples();
            Thread plowThread = new Thread(plowSamples);

            plowThread.start();
        }
        else if(intakeEncoderPosition < 950) {
            plowUp = true;
            plowServo.setPosition(0.82);
        }
        if (runPlow) {
            oldTouch = true;
        }
        else{
            oldTouch = false;
        }

    }

    // Parameter for assent motors
    LeftAssentControllerParams leftAssentControllerParams = new LeftAssentControllerParams();
    PIDF_Controller leftAssentController;

    RightAssentControllerParams rightAssentControllerParams = new RightAssentControllerParams();
    PIDF_Controller rightAssentController;

    boolean assentInti = false;
    double leftAssentPosition;
    double rightAssentPosition;

    // Control the ascent motors with pidf
    public void assent_control(boolean ascend_button, boolean descend_button) {
        if (!assentInti) {
            leftAssentController = new PIDF_Controller.Builder()
                    .setControllerMotor(leftAssentMotor)
                    .setControllerParams(leftAssentControllerParams.params)
                    .setMaxSpeed(1)
                    .setEncoderPosition(leftAssentEncoderPosition)
                    .setStopOnTargetReached(false)
                    .build();

            rightAssentController = new PIDF_Controller.Builder()
                    .setControllerMotor(rightAssentMotor)
                    .setControllerParams(rightAssentControllerParams.params)
                    .setMaxSpeed(1)
                    .setEncoderPosition(rightAssentEncoderPosition)
                    .setStopOnTargetReached(false)
                    .build();

            assentInti = true;
        }

        if (ascend_button) {
            leftAssentPosition = 14;
            rightAssentPosition = 14;
        } else if (descend_button) {
            leftAssentPosition = 0;
            rightAssentPosition = 0;
        }

        leftAssentController.extendTo(leftAssentPosition);
        rightAssentController.extendTo(rightAssentPosition);

        leftAssentController.loopController();
        rightAssentController.loopController();
    }

    // Parameters for scoring slides
    ArmControllerParams armControllerParams = new ArmControllerParams();
    PIDF_Controller armController;

    IntakeControllerParams intakeControllerParams = new IntakeControllerParams();
    PIDF_Controller intakeController;

    boolean slideInit = false;
    double armPosition = 0;
    double intakePosition = 0;

    // controls lhe slides for scoring
    public void slide_control(double intake_slide_stick, double lifter_slide_stick, double deadZone) {

        if (!slideInit) {
            armController = new PIDF_Controller.Builder()
                    .setControllerMotor(armMotor)
                    .setControllerParams(armControllerParams.params)
                    .setMaxSpeed(1)
                    .setEncoderPosition(armEncoderPosition)
                    .setStopOnTargetReached(false)
                    .build();

            intakeController = new PIDF_Controller.Builder()
                    .setControllerMotor(intakeMotor)
                    .setControllerParams(intakeControllerParams.params)
                    .setMaxSpeed(1)
                    .setEncoderPosition(intakeEncoderPosition)
                    .setStopOnTargetReached(false)
                    .build();

            slideInit = true;
        }

        if (gamepad2.dpad_up) {
            armPosition = 16;
            armPivot.setPosition(0.1);
        } else if (gamepad2.dpad_down) {
            sampleServo.setPosition(0.85);
            armPosition = 10;


        }

        if (lifter_slide_stick > deadZone) {
            armPosition = 37;
        } else if (lifter_slide_stick < -deadZone) {
            armPosition = 0;
        }

        if (gamepad2.a) {
            armController.runController(false);
            if (abs(lifter_slide_stick) > deadZone) {
                armMotor.setPower(lifter_slide_stick);
            } else {
                armMotor.setPower(0);
            }
            if (gamepad2.dpad_down) {
                armPosition = 7;
            }
        } else {
            armController.runController(true);
            armController.extendTo(armPosition);
            armController.loopController();
            if(isIntakeSlideAuto == true){
                intakeController.runController(true);
                intakeController.extendTo(intakePosition);
                intakeController.loopController();
            }
            else{
                intakeController.runController(false);
            }
        }

        // Intake slide power
        if (abs(intake_slide_stick) > deadZone) {
            if(isIntakeSlideAuto == false){
                intakeMotor.setPower(-intake_slide_stick);
            }

            if (oldInSlide == false) {
                isIntakeSlideAuto = false;
            }
            oldInSlide = true;
        } else if(isIntakeSlideAuto == false) {
            intakeMotor.setPower(0);
            oldInSlide = false;
        }
    }

    public void red_intake_control(double intake_wheels_in,
                                   double intake_wheels_out,
                                   boolean intake_pivot_up,
                                   boolean intake_pivot_down,
                                   boolean intake_pivot_reset,
                                   double deadZone) {
        loopSensor();

        if (!isUp && sensorDistance > 3.5 && sensorDistance < 4.5) {
            intake_wheel_power = -0.3;
        } else if (intake_wheels_out > deadZone) {
            intake_wheel_power = 1;
        } else if (intake_wheels_in > deadZone) {
            intake_wheel_power = -1;
        } else if (intakePivot.getPosition() < 0.3) {
            intake_wheel_power = 1;
        } else {
            intake_wheel_power = 0;
        }

        if (intake_pivot_up) {//transfer
            if(transfer == false) {
                if (armPosition < 1 &&  armController.targetReached && armPivot.getPosition() == 0.8 && intakePosition <= 0.5 && isIntakeSlideAuto == true  ) {
                    isIntakeSlideAuto = false;
                    TransferSample transferSample = new TransferSample();
                    Thread transferThread = new Thread(transferSample);

                    transferThread.start();


                } else {
                    armPosition = 0;
                    sampleServo.setPosition(0.65);
                    armPivot.setPosition(0.8);
                    isIntakeSlideAuto = true;
                    intakePosition = 0;
                    flashRed();
                }
            }


        } else if (intake_pivot_down) {
            intakePivot.setPosition(0.12);
            intake_wheel_power = 1;
            isUp = false;
            transfer = false;
            isIntakeSlideAuto  = false;
        } else if (intake_pivot_reset) {
            intakePivot.setPosition(0.5);
            isUp = true;
            transfer = false;
            isIntakeSlideAuto  = false;
        } else if (!isUp && sensorDistance < 1 && red || !isUp && sensorDistance < 1 && yellow) {
            intakePivot.setPosition(0.5);
            doingIntake = true;
            isUp = true;
            isIntakeSlideAuto = true;
            intakePosition = 0;
            blinkGreen();
        } else if (!isUp && sensorDistance < 1 && blue) {
            intakePivot.setPosition(0.31);
            intake_wheel_power = -1;
            flashRed();
        }
    }

    public void blue_intake_control(double intake_wheels_in,
                                    double intake_wheels_out,
                                    boolean intake_pivot_up,
                                    boolean intake_pivot_down,
                                    boolean intake_pivot_reset,
                                    double deadZone) {

        loopSensor();

        if (!isUp && sensorDistance > 3.5 && sensorDistance < 4.5) {
            intake_wheel_power = -0.3;
        } else if (intake_wheels_out > deadZone) {
            intake_wheel_power = 1;
        } else if (intake_wheels_in > deadZone) {
            intake_wheel_power = -1;
        } else if (intakePivot.getPosition() < 0.3) {
            intake_wheel_power = 1;
        } else {
            intake_wheel_power = 0;
        }

        if (intake_pivot_up) {
            if(transfer == false) {
                if (armPosition < 1 && armController.targetReached && armPivot.getPosition() == 0.8 && intakePosition <= 0.5 && isIntakeSlideAuto == true  ) {
                    isIntakeSlideAuto = false;
                    TransferSample transferSample = new TransferSample();
                    Thread transferThread = new Thread(transferSample);

                    transferThread.start();


                } else {
                    armPosition = 0;
                    sampleServo.setPosition(0.65);
                    armPivot.setPosition(0.8);
                    isIntakeSlideAuto = true;
                    intakePosition = 0;
                    flashRed();
                }
            }
        } else if (intake_pivot_down) {
            intakePivot.setPosition(0.12);
            intake_wheel_power = 1;
            isUp = false;
            transfer = false;
            isIntakeSlideAuto  = false;
        } else if (intake_pivot_reset) {
            intakePivot.setPosition(0.5);
            isUp = true;
            transfer = false;
            isIntakeSlideAuto  = false;
        } else if (!isUp && sensorDistance < 1 && blue || !isUp && sensorDistance < 1 && yellow) {
            intakePivot.setPosition(0.5);
            doingIntake = true;
            isUp = true;
            isIntakeSlideAuto = true;
            intakePosition = 0;
            blinkGreen();
        } else if (!isUp && sensorDistance < 1 && red) {

            intakePivot.setPosition(0.31);
            intake_wheel_power = -1;
            flashRed();

        }
    }

    public void doIntake() {//depreciated
        if (intakeController.targetReached) {
            intakePivot.setPosition(0.71);
            isUp = true;

        }
    }

    // Power Sets
    public void servo_power_sets() {
        if (!transfer) {
            left.setPower(intake_wheel_power);
            right.setPower(intake_wheel_power);
        }
    }

    public void writeRobotInfo() {
        drive.updatePoseEstimate();

        armEncoderPosition = armMotor.getCurrentPosition();
        intakeEncoderPosition = intakeMotor.getCurrentPosition();
        leftAssentEncoderPosition = leftAssentMotor.getCurrentPosition();
        rightAssentEncoderPosition = rightAssentMotor.getCurrentPosition();
        robotPoseX = drive.pose.position.x;
        robotPoseY = drive.pose.position.y;
        robotHeading = Math.toDegrees(drive.pose.heading.toDouble());

        try {
            fileReadWriter.writeFile(
                    armEncoderPosition,
                    intakeEncoderPosition,
                    leftAssentEncoderPosition,
                    rightAssentEncoderPosition,
                    robotPoseX,
                    robotPoseY,
                    robotHeading
            );
        } catch (IOException e) {
            // Nothing
        }
    }

    // Telemetry for Debugging
    public void telemetry() {
        drive.updatePoseEstimate();
        telemetry.addLine("-----intake slide-----");
        telemetry.addData("intake postion", intakePosition);
        telemetry.addData("intake slide position", intakeMotor.getCurrentPosition());
        telemetry.addData("intake slide power", intakeMotor.getPower());
        telemetry.addData("intake slide auto", isIntakeSlideAuto);
        telemetry.addLine("-----color distance sensor-----");
        telemetry.addData("distance", ((DistanceSensor) sensor).getDistance(DistanceUnit.CM));
        telemetry.addData("red", sensor.red());
        telemetry.addData("green", sensor.green());
        telemetry.addData("blue", sensor.blue());
        telemetry.addData("red", red);
        telemetry.addData("yellow", yellow);
        telemetry.addData("blue", blue);
        telemetry.addLine("-----intake assembly-----");
        telemetry.addData("intake slide position", intakeMotor.getCurrentPosition());
        telemetry.addData("intake wheel power", intake_wheel_power);
        telemetry.addData("intake wrist position", intakePivot.getPosition());
        telemetry.addData("plow position", plowServo.getPosition());
        telemetry.addData("IsUp", isUp);
        telemetry.addLine("-----lifter assembly-----");
        telemetry.addData("lifter slide position", armMotor.getCurrentPosition());
        telemetry.addData("output trough position", armPivot.getPosition());
        telemetry.addData("specimen claw position", sampleServo.getPosition());
        telemetry.addData("IMU", imu.getRobotYawPitchRollAngles());
        telemetry.addData("Robot Pose X", drive.pose.position.x);
        telemetry.addData("Robot Pose Y", drive.pose.position.y);
        telemetry.addData("Robot Pose Heading", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("loop time", time.milliseconds());
        telemetry.update();
        time.reset();
    }

    // Sensor Utilities
    public boolean red = false;
    public boolean yellow = false;
    public boolean blue = false;

    public double sensorDistance;

    public void loopSensor() {
        sensorDistance = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);

        if (sensor.red() > 4000 && sensor.green() > 6000) {
            yellow = true;
        } else if (sensor.blue() > 3000) {
            blue = true;
        } else if (sensor.red() > 3000) {
            red = true;
        } else {
            red = false;
            yellow = false;
            blue = false;
        }
    }

    class TransferSample implements Runnable {

        @Override
        public void run() {
            try {
                transfer = true;
                armPivot.setPosition(0.8);
                TimeUnit.MILLISECONDS.sleep(500);
                sampleServo.setPosition(0.65);
                intakePivot.setPosition(0.73);
                TimeUnit.MILLISECONDS.sleep(500);
                left.setPower(-0.3);
                right.setPower(-0.3);
                TimeUnit.MILLISECONDS.sleep(250);
                sampleServo.setPosition(0.78);
                left.setPower(0);
                right.setPower(0);
                TimeUnit.MILLISECONDS.sleep(300);
                intakePivot.setPosition(0.5);
                transfer = false;
                isUp = true;
                TimeUnit.MILLISECONDS.sleep(500);
                armPosition = 37;

            } catch (InterruptedException e) {
                // Nothing
            }
        }
    }
    class OutputSample implements Runnable {

        @Override
        public void run() {
            try {
                armPivot.setPosition(0.1);
                TimeUnit.MILLISECONDS.sleep(900);
                sampleServo.setPosition(0.65);
                TimeUnit.MILLISECONDS.sleep(300);
                armPivot.setPosition(0.8);
                TimeUnit.MILLISECONDS.sleep(900);
                blinkGreen();
                sampleServo.setPosition(0.65);
                armPivot.setPosition(0.8);
            } catch (InterruptedException e) {

            }
        }
    }

        class PlowSamples implements Runnable {
        @Override
        public void run() {
            try {

                    if (plowUp) {
                        plowUp = false;
                        plowServo.setPosition(0.2);
                        TimeUnit.MILLISECONDS.sleep(50);
                    } else {
                        plowUp = true;
                        plowServo.setPosition(0.82);
                        TimeUnit.MILLISECONDS.sleep(50);
                    }

            } catch (InterruptedException e) {
                // Nothing
            }
        }
    }
    class FlashRed implements Runnable {
        @Override
        public void run() {
            try {
                revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                TimeUnit.MILLISECONDS.sleep(750);
                revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);


            } catch (InterruptedException e) {
                // Nothing
            }
        }
    }
    public void flashRed(){
        FlashRed flashred = new FlashRed();
        Thread redThread = new Thread(flashred);

        redThread.start();
    }
    class BlinkGreen implements Runnable {
        @Override
        public void run() {
            try {
                revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                TimeUnit.MILLISECONDS.sleep(3000);
                revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);


            } catch (InterruptedException e) {
                // Nothing
            }
        }
    }
    public void blinkGreen(){
        BlinkGreen blinkgreen = new BlinkGreen();
        Thread greenThread = new Thread(blinkgreen);

        greenThread.start();
    }
}
