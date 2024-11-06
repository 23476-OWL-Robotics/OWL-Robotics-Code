package org.firstinspires.ftc.teamcode.TeleOp;

//imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Utilities.TeleOp.FieldCentric.*;

@TeleOp(name = "FieldCentric")
public class FieldCentric extends LinearOpMode {
    //add other files
    public FCdrivecontrol drive = new FCdrivecontrol();
    public FCarmscontrol arms = new FCarmscontrol();
    public FCcommon common = new FCcommon();


    @Override
    public void runOpMode() {

        // run the hardware maps
        common.hardwareMaps();

        // Put initialization blocks here.

        //motor setup
        drive.driveMotorSetup();
        arms.armMotorSetup();

        //IMU setup
        drive.set_up_imu();


        waitForStart();
        // inital variable values
        drive.rotX = 0;
        drive.rotY = 0;
        drive.heading_divisoin = 1;
        arms.intakeWristPos = 1;

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                // turn gamepad values to variables
                common.gamepadToVariables(1);

                //drive controls
                drive.rotate_X_and_Y();
                drive.drive_control();
                drive.motor_power_sets();

                //arms control
                arms.armControl();

                //hang control
                drive.hangControl();

                //telemetry
                common.useTelemetry();


            }
        }

    }

}