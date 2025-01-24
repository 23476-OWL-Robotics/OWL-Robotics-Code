package org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams;

public class RightAssentControllerParams {

    double inPerTick = 0.0062277580071174;

    double p = 0.0023;
    double i = 0.0;
    double d = 0.0;
    double f = 0.0;

    public ControllerParams params = new ControllerParams(p, i, d, f, inPerTick);
}
