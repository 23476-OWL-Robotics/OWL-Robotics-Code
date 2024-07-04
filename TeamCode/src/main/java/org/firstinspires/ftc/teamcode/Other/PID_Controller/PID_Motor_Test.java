package org.firstinspires.ftc.teamcode.Other.PID_Controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class PID_Motor_Test extends LinearOpMode {

    DcMotor motor;

    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "motor");

        PID_Controller controller = new PID_Controller(motor);

        waitForStart();
        if (opModeIsActive()) {

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (motor.getCurrentPosition() > -10 && motor.getCurrentPosition() < 10) {
                controller.rotateTo(0.5);
            } else {
                controller.rotateTo(0);
            }
        }
    }
}
