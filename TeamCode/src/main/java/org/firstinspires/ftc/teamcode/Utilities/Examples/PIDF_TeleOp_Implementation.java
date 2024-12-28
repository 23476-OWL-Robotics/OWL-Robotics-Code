package org.firstinspires.ftc.teamcode.Utilities.Examples;

/*
    This is an example class to show how to implement the PIDF_Controller class into a TeleOp Program
    This program uses ExampleControllerParams

    As this program is not meant to be run, please ignore setting motor names, directions, etc.

    There are several usages of PIDF_Controller that are not used here.
*/

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.PIDF_Controller;

@TeleOp
@Disabled
public class PIDF_TeleOp_Implementation extends LinearOpMode {

    // Rotation / Extension Motor
    DcMotorEx motor;

    @Override
    public void runOpMode() {

        // Set the motor name
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        // Add ExampleControllerParams as controllerParams
        ExampleControllerParams controllerParams = new ExampleControllerParams();

        // Add PIDF_Controller as controller
        // Add the controller parameters and motor you want to loop.
        // Set the maxSpeed and if you want the controller to stopOnTargetReached
        PIDF_Controller controller = new PIDF_Controller.Builder()
                .setControllerMotor(motor)
                .setControllerParams(controllerParams.params)
                .setMaxSpeed(1)
                .setStopOnTargetReached(false)
                .build();

        waitForStart();
        if (opModeIsActive()) {

            // Tell the motor to use the encoder
            // This is required for the controller
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive()) {

                // To run the controller, use either rotateTo, extendTo, or retractTo
                // Below, I am telling the controller to rotate 3 times in the positive direction
                controller.rotateTo(3);

                // If your motor is on a slider, you can use extendTo
                // Below, I am telling the controller to extend the slides 1 inch
                controller.extendTo(1);

                // If you wish to pause the controller in the middle of the program, use runController
                controller.runController(false);
                controller.runController(true);

                // This will loop the controller every time the program loops.
                // This function is required in every program to run the controller
                controller.loopController();
            }
        }
    }
}
