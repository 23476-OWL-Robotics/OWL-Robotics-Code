package org.firstinspires.ftc.teamcode.Util.PIDFController;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PositionController {

    // Current State and EndState
    volatile ControllerStates state;
    volatile ControllerStates endState;

    // create ElapsedTime
    ElapsedTime timer = new ElapsedTime();
    DcMotorEx motor;

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

    double motorPower;

    int motorOffset = 0;

    // Motor Coefficients
    Coefficients.PositionCoefficients coefficients;

    Thread thread;
    PIDThread pidThread;

    // Controller constructor class
    private PositionController(Builder builder) {
        state = ControllerStates.STOP_CONTROLLER;
        endState = builder.endState;
        motor = builder.motor;

        this.coefficients = builder.coefficients;

        this.endErrorValue = builder.endErrorValue;

        pidThread = new PIDThread();
        thread = new Thread(pidThread);
    }

    // Builder Class
    public static class Builder {
        double endErrorValue;
        Coefficients.PositionCoefficients coefficients;
        ControllerStates endState;
        DcMotorEx motor;

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

        public Builder setMotor(DcMotorEx motor) {
            this.motor = motor;
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
            if (motor == null) {
                throw new IllegalArgumentException("You Must Set a Motor for the Controller");
            }

            return new PositionController(this);
        }
    }

    public void startController() {
        setState(ControllerStates.RUN_CONTROLLER);
        pidThread.startThread();
    }

    public void stopController() {
        pidThread.stopThread();
        setState(ControllerStates.STOP_CONTROLLER);
    }

    public void terminateThread() {
        thread.interrupt();
    }

    public synchronized Thread.State getThreadState() {
        return thread.getState();
    }

    public synchronized long getThreadLoopTime() {
        return pidThread.getThreadLoopTime();
    }

    public synchronized void setMotorOffset(int o) {
        motorOffset = 0;
    }

    // Set the target position
    public synchronized void setTargetPosition(double target) {
        this.target = target;
    }

    // Returns the Target
    public synchronized double getTargetPosition() {
        return target;
    }

    // Returns the CurrentPosition
    public synchronized double getCurrentPosition() {
        return encoderPosition * coefficients.conversionUnit;
    }

    public synchronized double getMotorPosition() {
        return encoderPosition;
    }

    public synchronized double getMotorPower() {
        return motorPower;
    }

    // Set the controller state
    public synchronized void setState(ControllerStates state) {
        this.state = state;
    }

    // Will return the current state of the controller
    public synchronized ControllerStates getState() {
        return state;
    }

    // Will return the current endState of the controller
    public synchronized ControllerStates getEndState() {
        return endState;
    }

    // Will return the current encoder position
    public synchronized double getEncoderPosition() {
        return encoderPosition;
    }

    // Will return the position that the motor is being held at
    public synchronized double getHoldEncoderPosition() {
        return holdEncoderPosition;
    }

    // Will return the needed motor power
    public synchronized double getOut() {
        return Math.min(out, 0.8);
    }

    // Will return the controllers reference
    public synchronized double getReference() {
        return reference;
    }

    // Set the Coefficients
    public synchronized void setCoefficients(Coefficients.PositionCoefficients c) {
        this.coefficients = c;
    }

    // Will return the controller current coefficients
    public synchronized Coefficients.PositionCoefficients getCoefficients() {
        return coefficients;
    }

    // runController Loop
    private void runController(double encoderPosition) {

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

    class PIDThread implements Runnable {

        private boolean runThread = true;
        private long threadLoopTime = 0;
        Timer loopTime = new Timer();

        public void startThread() {
            if (thread.getState() == Thread.State.TERMINATED || thread.getState() == Thread.State.NEW) {
                runThread = true;
                thread.start();
            }
        }
        public synchronized void stopThread() {
            runThread = false;
        }

        public synchronized long getThreadLoopTime() {
            return threadLoopTime;
        }

        @Override
        public void run() {
            while (runThread) {
                runController(motor.getCurrentPosition() - motorOffset);
                motor.setPower(Math.min(out, 0.8));
                motorPower = motor.getPower();

                if (getState() == ControllerStates.STOP_CONTROLLER) {
                    stopThread();
                }

                if (Thread.currentThread().isInterrupted()) {
                    startThread();
                }

                threadLoopTime = loopTime.getElapsedTime();
                loopTime.resetTimer();
            }
            out = 0;
            motor.setPower(0);
        }
    }
}
