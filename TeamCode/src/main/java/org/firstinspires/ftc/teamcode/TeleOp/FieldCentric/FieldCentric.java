package org.firstinspires.ftc.teamcode.TeleOp.FieldCentric;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FieldCentric")
public class FieldCentric extends LinearOpMode {

    FCdrivecontrol drive = new FCdrivecontrol();
    FCarmscontrol arms = new FCarmscontrol();
    FCcommon common = new FCcommon();
    DcMotorEx armMotor;
    DcMotorEx intakeMotor;
    DcMotorEx leftAssentMotor;
    DcMotorEx rightAssentMotor;

    Servo specimenClaw;

/*
    public IMU imu_IMU;
    public DcMotor backLeftMotor;
    public DcMotor frontLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor frontRightMotor;

    YawPitchRollAngles myYawPitchRollAngles;
    public double front_left_power;
    public float X_stick;
    public double BotHeading;
    public double front_right_power;
    public double back_left_power;
    public double back_right_power;
    public float Y_stick;
    public double rotX;
    public double Turning;
    public int heading_divisoin;
    public double rotY;
*/


    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //leftAssentMotor = hardwareMap.get(DcMotorEx.class, "leftAssentMotor");
        //rightAssentMotor = hardwareMap.get(DcMotorEx.class, "rightAssentMotor");

        specimenClaw = hardwareMap.get(Servo.class, "specimenClaw");

        drive.imu_IMU = hardwareMap.get(IMU.class, "imu");
        drive.backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        drive.frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        drive.backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        drive.frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");



        // Put initialization blocks here.
        drive.set_up_imu();
        drive.motor_setup();
        waitForStart();
        drive.rotX = 0;
        drive.rotY = 0;
        drive.heading_divisoin = 1;
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                gamepadToVariables(1);
                drive.rotate_X_and_Y();
                drive.drive_control();
                drive.motor_power_sets();
                useTelemetry();
                armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
        }

    }
    public void useTelemetry(){

        telemetry.addData("x input", drive.X_stick);
        telemetry.addData("y input", drive.Y_stick);
        telemetry.addData("my yaw pitch roll", drive.myYawPitchRollAngles);
        telemetry.addData("BotHeading var", drive.BotHeading);
        telemetry.addData("turning", drive.Turning);
        telemetry.addData("rotY", drive.rotY);
        telemetry.addData("rotX", drive.rotX);
        telemetry.update();
    }
    public void gamepadToVariables(float turnmod){
        drive.X_stick = gamepad1.right_stick_x;
        drive.Y_stick = -gamepad1.right_stick_y;
        drive.Turning = gamepad1.left_stick_x * turnmod;
    }


}