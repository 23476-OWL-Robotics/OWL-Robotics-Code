package org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams;

public class ArmControllerParams {

    double inPerTick = 1; // inPerTick

    double p = 1;
    double i = 1;
    double d = 1;
    double f = 1;

    public ControllerParams params = new ControllerParams(p, i, d, f, inPerTick);
}
