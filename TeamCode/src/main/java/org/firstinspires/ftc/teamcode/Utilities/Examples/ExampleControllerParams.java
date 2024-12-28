package org.firstinspires.ftc.teamcode.Utilities.Examples;

import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.ControllerParams;

public class ExampleControllerParams {

    // ToDo: Set the rotPerTick to the value calculated in EncoderCounter
    //  This variable can also be inPerTick or degPerTick
    double rotPerTick = 0.0;

    // ToDo: Set the pidf values
    //  These are tuned in PIDF_Tuner
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
    double f = 0.0;

    // Add all of the values to ControllerParams
    public ControllerParams params = new ControllerParams(p, i, d, f, rotPerTick);
}
