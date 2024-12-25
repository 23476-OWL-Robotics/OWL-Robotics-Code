package org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller;

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

    // Max Speed
    double MAX_SPEED;

    // Create running boolean
    public boolean running = true;
    public boolean targetReached = false;
    private boolean stopOnTargetReached;

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

    // Set the max speed of the motor being controlled
    // Tetrix encoder values are only accurate at a speed of 0.8 or less
    public void setMaxSpeed(double MAX_SPEED) {
        this.MAX_SPEED = MAX_SPEED;
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
    public void loopController() {
        if (running) {
            PIDF_Calculator();
        }
    }

    // Create setStopOnTargetReached
    // This will determine if the controller will stop if the desired target is reached
    public void setStopOnTargetReached(boolean stop) {
        this.stopOnTargetReached = stop;
    }

    // Create runController
    // You can stop/start the controller from running if needed
    public void runController(boolean run) {
        running = run;
    }

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

        // reset the timer
        timer.reset();

        // get the reference
        reference = target / ConversionUnit;

        // obtain the encoder position
        encoderPosition = motor1.getCurrentPosition();
        // calculate the error
        error = reference - encoderPosition;

        // rate of change of the error
        derivative = (error - lastError); // / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        // feedForward
        feedForward = Math.cos(Math.toRadians(reference));

        // calculate the motor power
        out = (p * error) + (i * integralSum) + (d * derivative) + (f * feedForward);

        // set the motor power
        motor1.setPower(Math.min(out, MAX_SPEED));

        if (encoderPosition < reference + 5 && encoderPosition > reference -5) {
            targetReached = true;
            if (stopOnTargetReached) {
                motor1.setPower(0);
                running = false;
            }
        }

        lastError = error;

        timer.reset();
    }
}
