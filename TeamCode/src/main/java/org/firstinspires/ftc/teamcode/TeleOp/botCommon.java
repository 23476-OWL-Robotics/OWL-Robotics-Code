package org.firstinspires.ftc.teamcode.TeleOp;

//imports

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

    public DcMotorEx armMotor;
    public DcMotorEx intakeMotor;


    public Servo specimenClaw;
    public Servo intakePivot;

    public CRServo portIntake;
    public CRServo starbordIntake;

    public DistanceSensor blockDet_DistanceSensor;
    public ColorSensor blockDet;

    public IMU imu_IMU;
    public DcMotor backLeftMotor;
    public DcMotor frontLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor frontRightMotor;
    public DcMotorEx leftAssentMotor;
    public DcMotorEx rightAssentMotor;

    //variable setups
    public YawPitchRollAngles myYawPitchRollAngles;
    public void hardwareMaps(){
        //map slide motors
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //arms.leftAssentMotor = hardwareMap.get(DcMotorEx.class, "leftAssentMotor");
        //arms.rightAssentMotor = hardwareMap.get(DcMotorEx.class, "rightAssentMotor");

        //map fixed servos
        specimenClaw = hardwareMap.get(Servo.class, "specimenClaw");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        //map continuos servos
        portIntake = hardwareMap.get(CRServo.class, "portIntake");
        starbordIntake = hardwareMap.get(CRServo.class, "starbordIntake");

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

    public void useTelemetry(){

        /*telemetry.addData("x input", X_stick);
        telemetry.addData("y input", Y_stick);
        telemetry.addLine("-----------------");
        telemetry.addData("my yaw pitch roll", myYawPitchRollAngles);
        telemetry.addData("BotHeading var", BotHeading);
        telemetry.addData("turning", Turning);
        telemetry.addData("rotY", rotY);
        telemetry.addData("rotX", rotX);
        telemetry.addLine("-----------------");
        telemetry.addData("spinner power", spinnerStick);
        telemetry.addData("intake wrist position", intakeWristPos);
        telemetry.addLine("-----------------");*/
        telemetry.addData("distance", blockDet_DistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("red", blockDet.red());
        telemetry.addData("green", blockDet.green());
        telemetry.addData("blue", blockDet.blue());
        telemetry.update();

    }
}
