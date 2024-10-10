package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MainDrive extends LinearOpMode {

    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    DcMotorEx motorUp;
    DcMotorEx motorOut;

    double x;
    double y;
    double ls;
    double rs;

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        motorUp = hardwareMap.get(DcMotorEx.class, "motorUp");
        motorOut = hardwareMap.get(DcMotorEx.class, "motorOut");

        waitForStart();
        if (opModeIsActive()) {

            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            motorUp.setDirection(DcMotorSimple.Direction.REVERSE);

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            while (opModeIsActive()) {

                x = gamepad1.right_stick_x;
                y = gamepad1.left_stick_y;
                ls = gamepad1.left_trigger;
                rs = gamepad1.right_trigger;

                if (gamepad1.right_bumper) {
                    if (y > 0.05 || y < -0.05 || x > 0.05 || x < -0.05) {
                        frontLeftMotor.setPower((y - x) * 0.3);
                        frontRightMotor.setPower((y + x) * 0.3);
                        backLeftMotor.setPower((y - x) * 0.3);
                        backRightMotor.setPower((y + x) * 0.3);
                    } else if (ls > 0.2 || rs > 0.1) {
                        frontLeftMotor.setPower((ls - rs) * 0.3);
                        frontRightMotor.setPower((-ls + rs) * 0.3);
                        backLeftMotor.setPower((-ls + rs) * 0.3);
                        backRightMotor.setPower((ls - rs) * 0.3);
                    } else {
                        frontLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        backLeftMotor.setPower(0);
                        backRightMotor.setPower(0);
                    }
                } else {
                    if (y > 0.05 || y < -0.05 || x > 0.05 || x < -0.05) {
                        frontLeftMotor.setPower(y - x);
                        frontRightMotor.setPower(y + x);
                        backLeftMotor.setPower(y - x);
                        backRightMotor.setPower(y + x);
                    } else if (ls > 0.2 || rs > 0.1) {
                        frontLeftMotor.setPower(ls - rs);
                        frontRightMotor.setPower(-ls + rs);
                        backLeftMotor.setPower(-ls + rs);
                        backRightMotor.setPower(ls - rs);
                    } else {
                        frontLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        backLeftMotor.setPower(0);
                        backRightMotor.setPower(0);
                    }
                }

                if (gamepad2.dpad_up) {
                    motorUp.setPower(0.5);
                } else if (gamepad2.dpad_down) {
                    motorUp.setPower(-0.7);
                } else {
                    motorUp.setPower(0);
                }

                if (gamepad2.dpad_left) {
                    motorOut.setPower(0.5);
                } else if (gamepad2.dpad_right) {
                    motorOut.setPower(-0.7);
                } else motorOut.setPower(0);
            }
        }
    }
}
