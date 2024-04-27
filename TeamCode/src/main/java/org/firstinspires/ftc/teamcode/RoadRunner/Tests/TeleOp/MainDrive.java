package org.firstinspires.ftc.teamcode.RoadRunner.Tests.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MainDrive extends LinearOpMode{

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public void runOpMode() {

        //ToDo: Set Robot Motor Names
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        //ToDo: Set Motor Directions
        //frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Create GamePad JoyStick doubles
        double y;
        double x;

        //Create GamePad Trigger doubles
        double l;
        double r;

        waitForStart();
        if (opModeIsActive()) {

            //Set Motors to Break
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            while (opModeIsActive()) {

                //Set doubles to GamePad Values
                y = gamepad1.left_stick_y;
                x = gamepad1.right_stick_x;
                l = gamepad1.left_trigger;
                r = gamepad1.right_trigger;

                //Drive Code
                if (y > 0.2 || y < -0.2 || x > 0.2 || x < -0.2) {
                    frontLeftMotor.setPower(y - x);
                    frontRightMotor.setPower(y + x);
                    backLeftMotor.setPower(y - x);
                    backRightMotor.setPower(y + x);
                } else if (l > 0.2 || l < -0.2 || r > 0.2 || r < -0.2) {
                    frontLeftMotor.setPower(-l + x);
                    frontRightMotor.setPower(l - x);
                    backLeftMotor.setPower(l - x);
                    backRightMotor.setPower(-l + x);
                } else {
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                }
            }
        }
    }
}
