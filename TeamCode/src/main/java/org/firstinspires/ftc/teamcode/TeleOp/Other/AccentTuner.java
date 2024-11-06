package org.firstinspires.ftc.teamcode.TeleOp.Other;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class AccentTuner extends LinearOpMode {

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

            rightAssentMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            while (opModeIsActive()) {
                if (gamepad2.dpad_up) {
                    leftAssentMotor.setPower(1);
                } else if (gamepad2.dpad_down) {
                    leftAssentMotor.setPower(-1);
                } else {
                    leftAssentMotor.setPower(0);
                }

                if (gamepad2.y) {
                    rightAssentMotor.setPower(1);
                } else if (gamepad2.a) {
                    rightAssentMotor.setPower(-1);
                } else {
                    rightAssentMotor.setPower(0);
                }
            }
        }
    }
}
