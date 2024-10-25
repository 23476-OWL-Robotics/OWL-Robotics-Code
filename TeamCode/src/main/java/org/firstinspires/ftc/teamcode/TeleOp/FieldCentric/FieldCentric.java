package org.firstinspires.ftc.teamcode.TeleOp.FieldCentric;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "FieldCentric")
public class FieldCentric extends LinearOpMode {


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
        FCdrivecontrol control = new FCdrivecontrol();

        control.imu_IMU = hardwareMap.get(IMU.class, "imu");
        control.backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        control.frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        control.backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        control.frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");

        // Put initialization blocks here.
        control.set_up_imu();
        control.motor_setup();
        waitForStart();
        control.rotX = 0;
        control.rotY = 0;
        control.heading_divisoin = 1;
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                control.X_stick = gamepad1.right_stick_x;
                control.Y_stick = -gamepad1.right_stick_y;
                control.Turning = gamepad1.left_stick_x * 1;
                //control.gamepad_to_variables();
                control.rotate_X_and_Y();
                control.drive_control();
                control.motor_power_sets();
                //control.telemetry2();
                telemetry.addData("x input", control.X_stick);
                telemetry.addData("y input", control.Y_stick);
                telemetry.addData("my yaw pitch roll", control.myYawPitchRollAngles);
                telemetry.addData("BotHeading var", control.BotHeading);
                telemetry.addData("turning", control.Turning);
                telemetry.addData("rotY", control.rotY);
                telemetry.addData("rotX", control.rotX);
                telemetry.update();
            }
        }
    }


}