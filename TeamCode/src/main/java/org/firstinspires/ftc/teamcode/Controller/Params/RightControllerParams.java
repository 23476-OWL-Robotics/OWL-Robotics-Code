package org.firstinspires.ftc.teamcode.Controller.Params;

public class RightControllerParams {

    double degPerTick = 0.3;

    double p = 0.0017;
    double i = 0.0;
    double d = 0.0;
    double f = 0.0;

    public ControllerParams params = new ControllerParams(p, i, d, f, degPerTick);
}
