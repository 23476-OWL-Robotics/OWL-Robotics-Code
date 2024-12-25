package org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams;

public class IntakeControllerParams {

    double inPerTick = 0.006371562;

    double p = 0.003;
    double i = 0.0;
    double d = 0.0;
    double f = 0.0;

    public ControllerParams params = new ControllerParams(p, i, d, f, inPerTick);
}
