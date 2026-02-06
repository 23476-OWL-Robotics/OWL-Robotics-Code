package org.firstinspires.ftc.teamcode.Util.PIDFController;

import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocityController {

    // Current State and EndState
    ControllerStates state;
    ControllerStates endState;

    // create ElapsedTime
    ElapsedTime timer = new ElapsedTime();

    // All variables for calculating out
    double reference;
    double integralSum;
    double lastError;
    double velocity;
    double error;
    double derivative;
    double target;
    double endErrorValue;
    double holdVelocity;
    double out;

    // Seconds Per Minute
    int secPerMin = 60;

    // Motor Coefficients
    Coefficients.VelocityCoefficients coefficients;

    private VelocityController(Builder b) {
        state = ControllerStates.STOP_CONTROLLER;
        endState = b.endState;

        this.coefficients = b.coefficients;

        this.endErrorValue = b.endErrorValue;
    }

    // Builder Class
    public static class Builder {
        double endErrorValue = 0;
        Coefficients.VelocityCoefficients coefficients;
        ControllerStates endState;

        // set the coefficients
        public Builder setCoefficients(double p, double i, double d, int ticksPerRev) {
            this.coefficients = new Coefficients.VelocityCoefficients(p, i, d, ticksPerRev);

            return this;
        }

        // other way to set coefficients
        public Builder setCoefficients(Coefficients.VelocityCoefficients coefficients) {
            this.coefficients = coefficients;

            return this;
        }

        // set the end position error
        public Builder setEndVelocityError(double endErrorValue) {
            this.endErrorValue = endErrorValue;

            return this;
        }

        // set the end state
        public Builder setEndState(ControllerStates endState) {
            this.endState = endState;
            return this;
        }

        // build
        public VelocityController build() {

            if (coefficients == null) {
                throw new IllegalArgumentException("You Must Set the PIDF Coefficients");
            }
            if (endState == null) {
                throw new IllegalArgumentException("You Must Set an End State for the Controller");
            }

            return new VelocityController(this);
        }
    }

    public void runController(double velocity) {
        this.velocity = velocity;

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
                holdVelocity = velocity;
                break;
        }
    }

    // Set the target position
    public void setTargetVelocity(double targetVel) {
        this.target = targetVel;
    }
    public void setTargetRPM(int targetRPM) {
        this.target = (double) (coefficients.ticksPerRev * targetRPM) / secPerMin;
    }

    // Sets the Controller State
    public void setState(ControllerStates state) {
        this.state = state;
    }

    // Returns the Current State
    public ControllerStates getState() {
        return state;
    }

    // Returns the End State
    public ControllerStates getEndState() {
        return endState;
    }

    // Returns the Velocity
    public double getCurrentVelocity() {
        return velocity;
    }

    // Returns the RPM
    public double getCurrentRPM() {
        return (velocity / coefficients.ticksPerRev) * secPerMin;
    }

    // Returns the Target Velocity
    public double getTargetVelocity() {
        return target;
    }

    // Returns the Target Velocity in RPM
    public double getTargetRPM() {
        return (target / coefficients.ticksPerRev) * secPerMin;
    }

    // Returns the position that the motor is being held at
    public double getHoldVelocity() {
        return holdVelocity;
    }

    // Returns the needed motor power
    public double getOut() {
        return out;
    }

    // Returns the controllers reference
    public double getReference() {
        return reference;
    }

    // Set the Coefficients
    public void setCoefficients(Coefficients.VelocityCoefficients c) {
        this.coefficients = c;
    }

    // Returns the controller current coefficients
    public Coefficients.VelocityCoefficients getCoefficients() {
        return coefficients;
    }

    private void calculate() {
        // get the reference
        reference = target;

        // calculate the error
        error = reference - velocity;

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        // rate of change of the error
        derivative = (error - lastError) / timer.seconds();

        // calculate the motor power
        out += Math.min((coefficients.p * error) + (coefficients.i * integralSum) + (coefficients.d * derivative), 0.2);
        out = Math.min(out, 1);

        holdVelocity = velocity;

        // save error
        lastError = error;

        // sets the controller state to endState when the controller is within an acceptable velocity from target
        if (velocity < (reference + endErrorValue) && velocity > (reference - endErrorValue)) {
            state = endState;
            timer.reset();

            return;
        }

        // reset timer
        timer.reset();
    }

    private void hold() {

    }
}
