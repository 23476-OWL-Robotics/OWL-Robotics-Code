package org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams;

public class ControllerParams {

    public double ConversionUnit;

    public double p;
    public double i;
    public double d;
    public double f;

    public ControllerParams(double p, double i, double d, double f, double ConversionUnit) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;

        this.ConversionUnit = ConversionUnit;
    }
}
