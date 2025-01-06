package org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams;

public class LeftAssentControllerParams {

    double inPerTick = 0.000893541;

    double p = 0.0065;
    double i = 0.0;
    double d = 0.0;
    double f = 0.0;

    public ControllerParams params = new ControllerParams(p, i, d, f, inPerTick);
}
