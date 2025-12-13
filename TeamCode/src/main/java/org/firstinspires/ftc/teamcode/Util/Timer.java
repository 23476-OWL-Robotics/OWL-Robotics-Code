package org.firstinspires.ftc.teamcode.Util;

public class Timer {

    double elapsedTime = 0;
    double oldTime = 0;
    double waitingTime;

    boolean finished = true;

    long secondToNano = 1000000000;
    long milliToNano = 1000000;

    public void loop() {
        elapsedTime = System.nanoTime() - oldTime;

        if (waitingTime > 0) {
            waitingTime -= elapsedTime;
        } else {
            waitingTime = 0;
            finished = true;
        }

        oldTime = System.nanoTime();
    }

    public void setSecondTimer(double seconds) {
        finished = false;
        waitingTime = seconds * secondToNano;
    }
    public void setMillisecondTimer(double milliseconds) {
        finished = false;
        waitingTime = milliseconds * milliToNano;
    }

    public double getSecondsRemaining() {
        return waitingTime / secondToNano;
    }
    public double getMillisecondsRemaining() {
        return waitingTime / milliToNano;
    }

    public boolean isFinished() {
        return finished;
    }

    public double getWaitingTime() {
        return waitingTime;
    }

    public double getElapsedTime() {
        return elapsedTime;
    }
}
