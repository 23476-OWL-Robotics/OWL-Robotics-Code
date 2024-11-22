package org.firstinspires.ftc.teamcode.TeleOp.mainProgram;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TeleOp.botCommon;


public class driveFunctions extends botCommon{

    public void D_Control(float X_in, float Y_in, float Turn_in){
        rotate_X_and_Y(X_in, Y_in, -1);
        drive_control(Turn_in);

    }
    // the field centric change code
    public void rotate_X_and_Y(float initX, float initY, int Head_div) {
        orientation = imu_IMU.getRobotYawPitchRollAngles();
        BotHeading = orientation.getYaw(AngleUnit.DEGREES) / Head_div;
        rotX = initX * Math.cos(-BotHeading / 180 * Math.PI) - initY * Math.sin(-BotHeading / 180 * Math.PI);
        rotY = initX * Math.sin(-BotHeading / 180 * Math.PI) + initY * Math.cos(-BotHeading / 180 * Math.PI);
    }

    //integrates variables to powers for motors
    public void drive_control(float Turn_in) {
        front_left_power = rotY + rotX + Turn_in;
        front_right_power = (rotY - rotX) - Turn_in;
        back_left_power = (rotY - rotX) + Turn_in;
        back_right_power = (rotY + rotX) - Turn_in;
    }
}
