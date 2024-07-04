package org.firstinspires.ftc.teamcode.Other.PIDF_Controller;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

class PIDF_Controller {

    public static class Params {
        double rotPerTick = 0.000695;
        double inPerTick = 0.0;

        double Kp = 0.0;
        double Ki = 0.0;
        double Kd = 0.0;
        double Kf = 0.0;
    }

    Params params = new Params();

    DcMotorEx motor1;

    public PIDF_Controller(DcMotorEx motor1) {
        this.motor1 = motor1;
    }


    public void rotateTo(double rotations) {

        double reference = rotations / params.rotPerTick;
        double integralSum = 0.0;
        double feedForward;
        double lastError = 0.0;

        int encoderPosition;
        double error;
        double derivative;
        double out;

        boolean setPointIsNotReached = true;
        ElapsedTime timer = new ElapsedTime();

        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        while (setPointIsNotReached) {

            // obtain the encoder position
            encoderPosition = motor1.getCurrentPosition();
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            feedForward = Math.cos(Math.toRadians(reference));

            out = (params.Kp * error) + (params.Ki * integralSum) + (params.Kd * derivative) + (params.Kf * feedForward);

            if (out > 0.6) {
                motor1.setPower(0.6);
            } else {
                motor1.setPower(out);
            }

            lastError = error;

            // reset the timer for next time
            timer.reset();

            if (encoderPosition <= reference + 1 && encoderPosition >= reference - 1) {
                setPointIsNotReached = false;
            }
        }
    }
}
