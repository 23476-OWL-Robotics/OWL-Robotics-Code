package org.firstinspires.ftc.teamcode.Util.PIDFController;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Creek.Red;

public class PositionController {

    // Current State and EndState
    ControllerStates state;
    ControllerStates endState;

    // create ElapsedTime
    ElapsedTime timer = new ElapsedTime();

    // All variables for calculating out
    double reference;
    double integralSum = 0.0;
    double lastError = 0.0;
    double encoderPosition;
    double error;
    double derivative;
    double feedForward;
    double target;
    double endErrorValue;
    double holdEncoderPosition;
    double out;

    // Motor Coefficients
    Coefficients.PositionCoefficients coefficients;

    // Controller constructor class
    private PositionController(Builder builder) {
        state = ControllerStates.STOP_CONTROLLER;
        endState = builder.endState;

        this.coefficients = builder.coefficients;

        this.endErrorValue = builder.endErrorValue;
    }

    // Builder Class
    public static class Builder {
        double endErrorValue;
        Coefficients.PositionCoefficients coefficients;
        ControllerStates endState;

        // set the coefficients
        public Builder setCoefficients(double p, double i, double d, double f, double conversionUnit) {
            this.coefficients = new Coefficients.PositionCoefficients(p, i, d, f, conversionUnit);

            return this;
        }

        // other way to set coefficients
        public Builder setCoefficients(Coefficients.PositionCoefficients coefficients) {
            this.coefficients = coefficients;

            return this;
        }

        // set the end position error
        public Builder setEndPositionError(double endErrorValue) {
            this.endErrorValue = endErrorValue;

            return this;
        }

        // set the end state
        public Builder setEndState(ControllerStates endState) {
            this.endState = endState;
            return this;
        }

        // build
        public PositionController build() {
            if (coefficients == null) {
                throw new IllegalArgumentException("You Must Set the PIDF Coefficients");
            }
            if (endState == null) {
                throw new IllegalArgumentException("You Must Set an End State for the Controller");
            }

            return new PositionController(this);
        }
    }

    // runController Loop
    public void runController(double encoderPosition) {

        // Set encoderPosition
        this.encoderPosition = encoderPosition;

        // switch state
        switch (state) {
            // The Controller rus to the set position
            case RUN_CONTROLLER:
                calculate();
                break;

            // The Controller keeps the motor at whatever position the motor is in when this state is set
            case HOLD_CONTROLLER:
                hold();
                break;

            // The Motor Power (out) is set to zero
            // The controller stops until its state is changed
            case STOP_CONTROLLER:
                out = 0;
                holdEncoderPosition = encoderPosition;
                break;
        }
    }

    // Set the target position
    public void setTargetPosition(double target) {
        this.target = target;
    }

    // Returns the Target
    public double getTargetPosition() {
        return target;
    }

    // Returns the CurrentPosition
    public double getCurrentPosition() {
        return encoderPosition * coefficients.conversionUnit;
    }

    // Set the controller state
    public void setState(ControllerStates state) {
        this.state = state;
    }

    // Will return the current state of the controller
    public ControllerStates getState() {
        return state;
    }

    // Will return the current endState of the controller
    public ControllerStates getEndState() {
        return endState;
    }

    // Will return the current encoder position
    public double getEncoderPosition() {
        return encoderPosition;
    }

    // Will return the position that the motor is being held at
    public double getHoldEncoderPosition() {
        return holdEncoderPosition;
    }

    // Will return the needed motor power
    public double getOut() {
        return Math.min(out, 0.8);
    }

    // Will return the controllers reference
    public double getReference() {
        return reference;
    }

    // Set the Coefficients
    public void setCoefficients(Coefficients.PositionCoefficients c) {
        this.coefficients = c;
    }

    // Will return the controller current coefficients
    public Coefficients.PositionCoefficients getCoefficients() {
        return coefficients;
    }

    // Calculates out
    private void calculate() {

        // get the reference
        reference = target / coefficients.conversionUnit;

        // calculate the error
        error = reference - encoderPosition;

        // rate of change of the error
        derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        // feedForward
        feedForward = Math.cos(Math.toRadians(reference));

        // calculate the motor power
        out = (coefficients.p * error) + (coefficients.i * integralSum) + (coefficients.d * derivative) + (coefficients.f * feedForward);

        // sets the controller state to endState when the controller is within an acceptable distance from target
        if (encoderPosition < (reference + endErrorValue) && encoderPosition > (reference - endErrorValue)) {
            holdEncoderPosition = encoderPosition;
            state = endState;

            timer.reset();

            return;
        }

        // save error
        lastError = error;

        // reset timer
        timer.reset();

        // set holdEncoderPosition
        holdEncoderPosition = encoderPosition;
    }

    // calculate the hold power
    private void hold() {

        if (encoderPosition < (reference + endErrorValue) && encoderPosition > (reference - endErrorValue)) {
            out = 0;
            return;
        } else {
            // set the reference
            reference = holdEncoderPosition;

            // calculate the error
            error = reference - encoderPosition;

            // motor power
            out = (coefficients.p * error);
        }
    }
}
