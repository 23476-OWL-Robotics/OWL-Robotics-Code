package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Utilities.TeleOp.FieldCentricUtil;

@TeleOp
public class FieldCentricRed extends FieldCentricUtil {


    @Override
    public void runOpMode() {

        hardwareMaps();

        initialize_motors();
        initializeServos();
        servo_power_sets();
        set_up_imu();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            initializeServos();
            servo_power_sets();
            while (opModeIsActive()) {
                rotate_x_and_y(gamepad1.left_stick_y, gamepad1.left_stick_x);
                drive_control(gamepad1.right_stick_x);
                assent_control(gamepad2.left_bumper, gamepad2.right_bumper);
                slide_control(gamepad2.right_stick_y, gamepad2.left_stick_y, gamepad2.touchpad, 0.2);
                red_intake_control(gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.touchpad, 0.1);
                sample_control(gamepad2.y, gamepad2.a);
                specimen_control(gamepad2.x, gamepad2.b);
                power_sets(gamepad1.right_bumper);
                Dointake();
                telemetry();
            }
        }
    }
}