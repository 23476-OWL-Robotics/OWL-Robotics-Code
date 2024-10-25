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

     FCdrivecontrol control = new FCdrivecontrol();

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



    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");

        // Put initialization blocks here.
        control.set_up_imu();
        control.motor_setup();
        waitForStart();
        rotX = 0;
        rotY = 0;
        heading_divisoin = 1;
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                control.gamepad_to_variables();
                control.rotate_X_and_Y();
                control.drive_control();
                control.motor_power_sets();
                control.telemetry2();

            }
        }
    }


}