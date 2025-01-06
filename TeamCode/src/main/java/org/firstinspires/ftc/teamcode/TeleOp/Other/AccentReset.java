package org.firstinspires.ftc.teamcode.TeleOp.Other;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class AccentReset extends LinearOpMode {

    DcMotorEx leftAssentMotor;
    DcMotorEx rightAssentMotor;

    @Override
    public void runOpMode() {

        leftAssentMotor = hardwareMap.get(DcMotorEx.class, "leftAssentMotor");
        rightAssentMotor = hardwareMap.get(DcMotorEx.class, "rightAssentMotor");

        waitForStart();
        if (opModeIsActive()) {

            leftAssentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightAssentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftAssentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightAssentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftAssentMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            while (opModeIsActive()) {
                if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                    leftAssentMotor.setPower(gamepad1.left_stick_y);
                } else {
                    leftAssentMotor.setPower(0);
                }

                if (Math.abs(gamepad1.right_stick_y) > 0.1) {
                    rightAssentMotor.setPower(gamepad1.right_stick_y);
                } else {
                    rightAssentMotor.setPower(0);
                }

                telemetry.addData("Left", leftAssentMotor.getCurrentPosition());
                telemetry.addData("Right", rightAssentMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
