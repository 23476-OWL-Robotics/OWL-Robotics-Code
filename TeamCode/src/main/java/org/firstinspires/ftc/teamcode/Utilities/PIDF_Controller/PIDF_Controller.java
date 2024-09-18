package org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.ControllerParams;

public class PIDF_Controller{

    // Parameters for Controller
    double ConversionUnit;
    double p;
    double i;
    double d;
    double f;

    // Controller target
    double target;

    // Create running boolean
    boolean running;

    // create ElapsedTime
    ElapsedTime timer = new ElapsedTime();

    // Create Motors
    DcMotorEx motor1;

    // PIDF_Controller
    public PIDF_Controller(ControllerParams params, DcMotorEx motor1) {
        this.ConversionUnit = params.ConversionUnit;

        this.p = params.p;
        this.i = params.i;
        this.d = params.d;
        this.f = params.f;

        this.motor1 = motor1;
    }

    // Create rotateTo, extendTo, and retractTo functions
    public void rotateTo(double rotations) {
        running = true;
        targetReached = false;
        this.target = rotations;
    }
    public void extendTo(double inches) {
        running = true;
        targetReached = false;
        this.target = inches;
    }
    public void retractTo(double inches) {
        running = true;
        targetReached = false;
        this.target = inches;
    }

    // Create runController function
    // Run this in the opModeIsActive while loop
    public void runController() {
        if (running) {
            PIDF_Calculator();
        }
    }

    // Create stopController function
    public void stopController() {
        running = false;
    }

    public boolean targetReached = false;


    // Create calculator variables and calculator function
    double reference;
    double integralSum = 0.0;
    double lastError = 0.0;
    double encoderPosition;
    double error;
    double derivative;
    double feedForward;
    double out;
    private void PIDF_Calculator() {

        timer.reset();

        reference = target / ConversionUnit;

        // obtain the encoder position
        encoderPosition = motor1.getCurrentPosition();
        // calculate the error
        error = reference - encoderPosition;

        // rate of change of the error
        derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        feedForward = Math.cos(Math.toRadians(reference));

        out = (p * error) + (i * integralSum) + (d * derivative) + (f * feedForward);

        motor1.setPower(Math.min(out, 0.6));

        if (encoderPosition < target + 5 && encoderPosition > target -5) {
            motor1.setPower(0);
            targetReached = true;
        }

        lastError = error;

        timer.reset();
    }
}
