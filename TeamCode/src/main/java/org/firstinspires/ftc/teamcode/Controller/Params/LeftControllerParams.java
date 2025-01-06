package org.firstinspires.ftc.teamcode.Controller.Params;

public class LeftControllerParams {

    double degPerTick = 0.0;

    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
    double f = 0.0;

    public ControllerParams params = new ControllerParams(p, i, d, f, degPerTick);
}
