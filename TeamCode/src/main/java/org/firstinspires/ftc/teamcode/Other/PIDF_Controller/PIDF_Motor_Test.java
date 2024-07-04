package org.firstinspires.ftc.teamcode.Other.PIDF_Controller;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDF_Motor_Test extends LinearOpMode {

    double rotPerTick = 0.000695;
    double target = 1.0;

    int reference = (int) (target / rotPerTick);

    double Kp = 0.00059;
    double Ki = 0.0;
    double Kd = 0.0;
    double Kf = 0.0;

    DcMotorEx motor;

    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "motor");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);

        waitForStart();
        if (opModeIsActive()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            motor.setTargetPosition(reference);
        }
    }
}
