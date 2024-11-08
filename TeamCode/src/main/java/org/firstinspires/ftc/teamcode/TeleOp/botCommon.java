package org.firstinspires.ftc.teamcode.TeleOp;

//imports

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class botCommon extends LinearOpMode {

    //viper motors
    public DcMotorEx armMotor;
    public DcMotorEx intakeMotor;

    //direct servos
    public Servo specimenClaw;
    public Servo intakePivot;

    //CR servos
    public CRServo left;
    public CRServo right;

    //sensors
    public DistanceSensor blockDet_DistanceSensor;
    public ColorSensor blockDet;

    //imu
    public IMU imu_IMU;
    public YawPitchRollAngles myYawPitchRollAngles;
    public void set_up_imu() {
        imu_IMU.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu_IMU.resetYaw();
    }

    //dive motors
    public DcMotor backLeftMotor;
    public DcMotor frontLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor frontRightMotor;

    //ascent motors
    public DcMotorEx leftAssentMotor;
    public DcMotorEx rightAssentMotor;

    //variable setups
    public double front_left_power;
    public double front_right_power;
    public double back_left_power;
    public double back_right_power;
    public double BotHeading;
    public double rotX;
    public double rotY;

    //map hardware devices
    public void hardwareMaps(){
        //map slide motors
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //arms.leftAssentMotor = hardwareMap.get(DcMotorEx.class, "leftAssentMotor");
        //arms.rightAssentMotor = hardwareMap.get(DcMotorEx.class, "rightAssentMotor");

        //map fixed servos
        specimenClaw = hardwareMap.get(Servo.class, "specimenClaw");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        //map continuous servos
        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "starboardIntake");

        //map sensors
        blockDet_DistanceSensor = hardwareMap.get(DistanceSensor.class, "blockDet");
        blockDet = hardwareMap.get(ColorSensor.class, "blockDet");

        //IMU map
        imu_IMU = hardwareMap.get(IMU.class, "imu");

        //map drive motors
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
    }

    //all telemetry values for testing
    public void useTelemetry(){

        telemetry.addLine("---encoder motors---");
        telemetry.addData("Arm: ", armMotor.getCurrentPosition());
        telemetry.addData("Intake: ", intakeMotor.getCurrentPosition());
        telemetry.addData("Left Assent: ", leftAssentMotor.getCurrentPosition());
        telemetry.addData("Right Assent: ", rightAssentMotor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addLine("---color/distance sensor---");
        telemetry.addData("distance", blockDet_DistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("red", blockDet.red());
        telemetry.addData("green", blockDet.green());
        telemetry.addData("blue", blockDet.blue());
        telemetry.addLine();
        telemetry.update();

    }

    public void initializeMotors(){
        //set zero powers
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftAssentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightAssentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set encoder modes
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftAssentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightAssentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set motor directions
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    //sends powers to drive motors
    public void drive_power_sets() {
        backLeftMotor.setPower(back_left_power);
        backRightMotor.setPower(back_right_power);
        frontLeftMotor.setPower(front_left_power);
        frontRightMotor.setPower(front_right_power);
    }
}
