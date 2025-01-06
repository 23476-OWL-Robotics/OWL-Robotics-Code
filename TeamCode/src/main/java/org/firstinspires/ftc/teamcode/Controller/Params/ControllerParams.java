package org.firstinspires.ftc.teamcode.Controller.Params;

public class ControllerParams {

    // Conversion unit for the Controller
    public double ConversionUnit;

    // pidf parameters
    public double p;
    public double i;
    public double d;
    public double f;

    // ControllerParams Constructor
    public ControllerParams(double p, double i, double d, double f, double ConversionUnit) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;

        this.ConversionUnit = ConversionUnit;
    }
}
