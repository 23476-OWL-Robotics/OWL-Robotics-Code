package org.firstinspires.ftc.teamcode.TeleOp.mainProgram;

public class armFunctions extends Main_TeleOp {

    // Ascent Buttons
    public void ascentControl(boolean ascend, boolean descend){

        if (ascend) {
            leftAssentMotor.setPower(1);
            rightAssentMotor.setPower(1);
        } else if (descend) {
            leftAssentMotor.setPower(-1);
            rightAssentMotor.setPower(-1);
        } else {
            leftAssentMotor.setPower(0);
            rightAssentMotor.setPower(0);
        }
    }

    public void slideControl(float liftStick, float intakeStick, double deadZone){
        lifterSlideControl(liftStick, deadZone);
        intakeSlideControl(intakeStick,deadZone);
    }

    //control the lift slide
    public void lifterSlideControl(float liftingStick, double deadZone){
        // Arm Slide Buttons
        if (liftingStick > deadZone) {
            armMotor.setPower(liftingStick);
        } else if (liftingStick < -deadZone) {
            armMotor.setPower(liftingStick);
        } else {
            armMotor.setPower(0);
        }
    }

    //control the intake slide
    public void intakeSlideControl(float slidingStick, double deadZone){
        if (slidingStick > deadZone) {
            intakeMotor.setPower(slidingStick);
        } else if (slidingStick < -deadZone) {
            intakeMotor.setPower(slidingStick);
        } else {
            intakeMotor.setPower(0);
        }
    }

    //intake control
    public void intakeControl(float manIn, float manOut, double deadZone){
        manSpinControl(manIn, manOut, deadZone);
        intakeMagicControl(deadZone);
    }

    //intake manual control
    public void manSpinControl(float intake, float outtake, double deadZone){
        // Intake Buttons
        if (intake > deadZone) {
            left.setPower(1);
            right.setPower(1);
        } else if (outtake > deadZone) {
            left.setPower(-1);
            right.setPower(-1);
        } else {
            left.setPower(0);
            right.setPower(0);
        }
    }

    //intake magic control
    public void intakeMagicControl(double deadZone){
        if (gamepad2.dpad_left) {
            intakePivot.setPosition(1);
        } else if (gamepad2.dpad_right) {
            intakePivot.setPosition(0.51);
        } else if (gamepad2.left_trigger > deadZone) {
            left.setPower(1);
            right.setPower(1);
        } else if (gamepad2.right_trigger > deadZone) {
            left.setPower(-1);
            right.setPower(-1);
        } else {
            if (intakeMotor.getCurrentPosition() < -500) {
                intakePivot.setPosition(0.51);
                left.setPower(1);
                right.setPower(1);
            } else {
                intakePivot.setPosition(1);
                left.setPower(0);
                right.setPower(0);
            }
        }
    }

    //specimen claw control
    public void specimenControl(boolean open, boolean close){
        if (close) {
            specimenClaw.setPosition(0.75);
        } else if (open) {
            specimenClaw.setPosition(1);
        }
    }
}
