package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TankDrive extends LinearOpMode {

    DcMotorEx leftMotor1;
    DcMotorEx leftMotor2;
    DcMotorEx rightMotor1;
    DcMotorEx rightMotor2;

    DcMotorEx threadMotor;

    Servo leftClaw;
    Servo rightClaw;

    double x, y;

    @Override
    public void runOpMode() {

        leftMotor1 = hardwareMap.get(DcMotorEx.class, "leftMotor1");
        leftMotor2 = hardwareMap.get(DcMotorEx.class, "leftMotor2");
        rightMotor1 = hardwareMap.get(DcMotorEx.class, "rightMotor1");
        rightMotor2 = hardwareMap.get(DcMotorEx.class, "rightMotor2");
        threadMotor = hardwareMap.get(DcMotorEx.class, "motor");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        waitForStart();
        if (opModeIsActive()) {

            leftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            threadMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

            threadMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            threadMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightClaw.setDirection(Servo.Direction.REVERSE);

            while (opModeIsActive()) {

                y = gamepad1.left_stick_y;
                x = gamepad1.right_stick_x;

                if (gamepad1.left_trigger > 0.2) {

                    if (y > 0.1 || y < -0.1 || x > 0.1 || x < -0.1) {
                        leftMotor1.setPower((y - x) * 0.3);
                        leftMotor2.setPower((y - x) * 0.3);
                        rightMotor1.setPower((y + x) * 0.3);
                        rightMotor2.setPower((y + x) * 0.3);
                    } else {
                        leftMotor1.setPower(0);
                        leftMotor2.setPower(0);
                        rightMotor1.setPower(0);
                        rightMotor2.setPower(0);
                    }
                } else {

                    if (y > 0.1 || y < -0.1 || x > 0.1 || x < -0.1) {
                        leftMotor1.setPower(y - x);
                        leftMotor2.setPower(y - x);
                        rightMotor1.setPower(y + x);
                        rightMotor2.setPower(y + x);
                    } else {
                        leftMotor1.setPower(0);
                        leftMotor2.setPower(0);
                        rightMotor1.setPower(0);
                        rightMotor2.setPower(0);
                    }
                }
            }
        }
    }
}
