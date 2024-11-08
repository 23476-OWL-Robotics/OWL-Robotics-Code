package org.firstinspires.ftc.teamcode.TeleOp.mainProgram;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.botCommon;

@TeleOp
public class Main_TeleOp extends botCommon {
 driveFunctions drive = new driveFunctions();
 armFunctions arm = new armFunctions();

    @Override
    public void runOpMode() throws InterruptedException {
        //ready motors for match
        hardwareMaps();
        initializeMotors();

        //turn on imu
        set_up_imu();

        while (opModeIsActive()) {
            //driving control
            drive.D_Control(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            //arms control
            arm.ascentControl(gamepad2.right_bumper,gamepad2.left_bumper);
            arm.slideControl(gamepad2.left_stick_y,gamepad2.right_stick_y,0.2);
            arm.intakeControl(gamepad2.left_trigger,gamepad2.right_trigger,0.2);
            arm.specimenControl(gamepad2.b,gamepad2.x);

            //set power to motors
            drive_power_sets();
            //output telemetry values
            useTelemetry();
        }



    }
}
