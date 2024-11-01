package org.firstinspires.ftc.teamcode.TeleOp.FieldCentric;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class FCcommon extends FieldCentric{

    // all telemetry things
    public void useTelemetry(){

        telemetry.addData("x input", drive.X_stick);
        telemetry.addData("y input", drive.Y_stick);
        telemetry.addLine("-----------------");
        telemetry.addData("my yaw pitch roll", drive.myYawPitchRollAngles);
        telemetry.addData("BotHeading var", drive.BotHeading);
        telemetry.addData("turning", drive.Turning);
        telemetry.addData("rotY", drive.rotY);
        telemetry.addData("rotX", drive.rotX);
        telemetry.addLine("-----------------");
        telemetry.addData("spinner power", arms.spinnerStick);
        telemetry.addData("intake wrist position", arms.intakeWristPos);
        telemetry.addLine("-----------------");
        telemetry.addData("distance", arms.blockDet_DistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("red", arms.blockDet.red());
        telemetry.addData("green", arms.blockDet.green());
        telemetry.addData("blue", arms.blockDet.blue());
        telemetry.update();

    }

    //tun gamepad valuse to variables
    public void gamepadToVariables(float turnmod){
        drive.X_stick = gamepad1.right_stick_x;
        drive.Y_stick = -gamepad1.right_stick_y;
        drive.Turning = gamepad1.left_stick_x * turnmod;
        arms.spinnerStick = gamepad2.left_stick_x;
        arms.intakeStick = gamepad2.left_stick_y;
    }

    // all hardware maps
    public void hardwareMaps(){
        //map slide motors
        arms.armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        arms.intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //arms.leftAssentMotor = hardwareMap.get(DcMotorEx.class, "leftAssentMotor");
        //arms.rightAssentMotor = hardwareMap.get(DcMotorEx.class, "rightAssentMotor");

        //map fixed servos
        arms.specimenClaw = hardwareMap.get(Servo.class, "specimenClaw");
        arms.intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        //map continuos servos
        arms.portIntake = hardwareMap.get(CRServo.class, "portIntake");
        arms. starbordIntake = hardwareMap.get(CRServo.class, "starbordIntake");

        //map sensors
        arms.blockDet_DistanceSensor = hardwareMap.get(DistanceSensor.class, "blockDet");
        arms.blockDet = hardwareMap.get(ColorSensor.class, "blockDet");

        //IMU map
        drive.imu_IMU = hardwareMap.get(IMU.class, "imu");

        //map drive motors
        drive.backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        drive.frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        drive.backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        drive.frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
    }

}
