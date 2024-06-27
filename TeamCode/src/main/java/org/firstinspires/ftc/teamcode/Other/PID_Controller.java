package org.firstinspires.ftc.teamcode.Other;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

class PID_Controller {

    public static class Params {
        double rotPerTick = 0.000695;
        double inPerTick = 0.0;

        double Kp = 0.0007;
        double Ki = 0.0001;
        double Kd = 0.000000001;
    }

    Params params = new Params();

    DcMotor motor1;

    public PID_Controller(DcMotor motor1) {
        this.motor1 = motor1;
    }


    public void rotateTo(double rotations) {

        double reference = rotations / params.rotPerTick;
        double integralSum = 0.0;
        double lastError = 0.0;

        int encoderPosition;
        double error;
        double derivative;
        double out;

        boolean setPointIsNotReached = true;
        ElapsedTime timer = new ElapsedTime();

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (setPointIsNotReached) {

            // obtain the encoder position
            encoderPosition = motor1.getCurrentPosition();
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            out = (params.Kp * error) + (params.Ki * integralSum) + (params.Kd * derivative);

            motor1.setPower(out);

            lastError = error;

            // reset the timer for next time
            timer.reset();

            if (encoderPosition <= reference + 10 && encoderPosition >= reference + 10) {
                setPointIsNotReached = false;
            }
        }
    }

    public void extendTo(double inches) {

        double reference = inches / params.inPerTick;
        double integralSum = 0.0;
        double lastError = 0.0;

        int encoderPosition;
        double error;
        double derivative;
        double out;

        boolean setPointIsNotReached = true;
        ElapsedTime timer = new ElapsedTime();

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (setPointIsNotReached) {

            // obtain the encoder position
            encoderPosition = motor1.getCurrentPosition();
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            out = (params.Kp * error) + (params.Ki * integralSum) + (params.Kd * derivative);

            motor1.setPower(out);

            lastError = error;

            // reset the timer for next time
            timer.reset();

            if (encoderPosition <= reference + 10 && encoderPosition >= reference + 10) {
                setPointIsNotReached = false;
            }
        }
    }

    public void retractTo(double inches) {

        double reference = inches / params.inPerTick;
        double integralSum = 0.0;
        double lastError = 0.0;

        int encoderPosition;
        double error;
        double derivative;
        double out;

        boolean setPointIsNotReached = true;
        ElapsedTime timer = new ElapsedTime();

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (setPointIsNotReached) {

            // obtain the encoder position
            encoderPosition = motor1.getCurrentPosition();
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            out = (params.Kp * error) + (params.Ki * integralSum) + (params.Kd * derivative);

            motor1.setPower(out);

            lastError = error;

            // reset the timer for next time
            timer.reset();

            if (encoderPosition <= reference + 10 && encoderPosition >= reference + 10) {
                setPointIsNotReached = false;
            }
        }
    }
}
