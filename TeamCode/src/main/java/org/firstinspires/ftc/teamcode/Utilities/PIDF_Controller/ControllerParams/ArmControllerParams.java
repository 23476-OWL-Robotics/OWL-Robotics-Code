package org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams;

public class ArmControllerParams {

    //double inPerTick = 0.006317227;  inPerTick

    double inPerTick = 0.008943679;  //inPerTick


    double p = 0.0033;
    double i = 0.0;
    double d = 0.0;
    double f = 0.0;

    public ControllerParams params = new ControllerParams(p, i, d, f, inPerTick);
}
