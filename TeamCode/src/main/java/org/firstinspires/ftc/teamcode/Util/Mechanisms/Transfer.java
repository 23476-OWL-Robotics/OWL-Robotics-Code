package org.firstinspires.ftc.teamcode.Util.Mechanisms;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Artifact;
import org.firstinspires.ftc.teamcode.Util.ArtifactPattern;
import org.firstinspires.ftc.teamcode.Util.ArtifactType;
import org.firstinspires.ftc.teamcode.Util.PIDFController.Coefficients;
import org.firstinspires.ftc.teamcode.Util.PIDFController.ControllerStates;
import org.firstinspires.ftc.teamcode.Util.PIDFController.PositionController;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.Utilities;

import java.util.ArrayList;

public class Transfer {

    public enum TransferState {
        Intake,
        Outtake
    }

    ArrayList<Artifact> artifacts = new ArrayList<>();
    ArrayList<ArtifactType> artifactPattern = new ArrayList<>();

    int patternIndex = 0;

    ColorSensor sensor1;
    ColorSensor sensor2;
    ColorSensor sensor3;

    DigitalChannel limitSwitch;

    Servo rotationServo;

    DcMotorEx liftMotor;

    Telemetry telemetry;
    OpMode opMode;

    HardwareMap hardwareMap;

    TransferState state;

    PositionController pController;

    Timer rotationTimer;

    final double LiftDown = 0.0;
    final double LiftUp = 5.0;

    final double RotationSlot_1_Intake = 0.0;
    final double RotationSlot_2_Intake = 0.4;
    final double RotationSlot_3_Intake = 0.8;

    final int RedLimit = 280;
    final int GreenLimit = 600;
    final int BlueLimit = 700;

    double RotationPosition = RotationSlot_1_Intake;

    int currentSlot;
    int artifactIndex;
    int liftMotorOffset;

    boolean canRotate = false;
    boolean isEjecting = false;
    boolean startedZero = true;
    boolean runZeroing = false;

    public Transfer(OpMode o) {
        this.opMode = o;
        this.hardwareMap = o.hardwareMap;
        this.telemetry = o.telemetry;

        rotationTimer = new Timer();
    }

    public void setPreloads(ArtifactType slot1, ArtifactType slot2, ArtifactType slot3) {
        artifacts.clear();
        artifacts.add(new Artifact(slot1, 1));
        artifacts.add(new Artifact(slot2, 2));
        artifacts.add(new Artifact(slot3, 3));
    }

    public void setPattern(ArtifactPattern p) {
        artifactPattern.clear();
        artifactPattern.add(p.getArtifact1());
        artifactPattern.add(p.getArtifact2());
        artifactPattern.add(p.getArtifact3());

        MoveToArtifact(artifactPattern.get(patternIndex));
    }

    public void init() {
        sensor1 = hardwareMap.get(RevColorSensorV3.class, "colorSensor1");
        sensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");
        sensor3 = hardwareMap.get(RevColorSensorV3.class, "colorSensor3");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        rotationServo = hardwareMap.get(Servo.class, "transferRotationServo");
        liftMotor = hardwareMap.get(DcMotorEx.class, "transferLiftMotor");

        Utilities.Set_PWM_Range(rotationServo, new PwmControl.PwmRange(500, 2500));

        rotationServo.setDirection(Servo.Direction.REVERSE);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (artifacts.isEmpty()) {
            artifacts.add(new Artifact(ArtifactType.EMPTY, 1));
            artifacts.add(new Artifact(ArtifactType.EMPTY, 2));
            artifacts.add(new Artifact(ArtifactType.EMPTY, 3));
        }

        pController = new PositionController.Builder()
                .setCoefficients(new Coefficients.PositionCoefficients.LiftMotorCoefficients())
                .setEndState(ControllerStates.STOP_CONTROLLER)
                .setEndPositionError(15)
                .build();

        currentSlot = 1;
        artifactIndex = 0;

        rotationServo.setPosition(RotationPosition);
    }

    public boolean ZeroLiftMotor() {
        if(limitSwitch.getState()) {
            liftMotor.setPower(0);
            liftMotorOffset = liftMotor.getCurrentPosition();
            return true;
        } else {
            liftMotor.setPower(-0.15);
            return false;
        }
    }

    public void ZeroLiftMotor_Running() {
        runZeroing = true;
        startedZero = true;
    }

    public void setState(TransferState state) {
        this.state = state;
    }

    public void switchState() {
        switch (state) {
            case Intake: state = TransferState.Outtake; MoveToArtifact(artifactPattern.get(patternIndex)); break;
            case Outtake: state = TransferState.Intake; MoveToArtifact(ArtifactType.EMPTY); break;
        }
    }

    public TransferState getState() {
        return state;
    }

    public void EjectSelectedArtifact() {
        if (CanMove()) {
            pController.setTargetPosition(LiftUp);
            pController.setState(ControllerStates.RUN_CONTROLLER);
            isEjecting = true;

            artifacts.set(artifactIndex, new Artifact(ArtifactType.EMPTY, currentSlot));

            if (patternIndex < 2) {
                patternIndex++;
            } else {
                patternIndex = 0;
            }

            canRotate = true;
        }
    }

    public void loop() {
        rotationTimer.loop();

        pController.runController((liftMotor.getCurrentPosition() - liftMotorOffset));

        if (pController.getState() == ControllerStates.STOP_CONTROLLER && isEjecting) {
            pController.setTargetPosition(LiftDown);
            pController.setState(ControllerStates.RUN_CONTROLLER);
            isEjecting = false;
        }

        switch (state) {
            case Intake: {

                if (CanMove()) {

                    if (runZeroing) {
                        if (startedZero) {
                            if (limitSwitch.getState()) {
                                liftMotor.setPower(0.15);
                            } else if (!limitSwitch.getState()) {
                                liftMotor.setPower(0);
                                startedZero = false;
                            }
                        } else {

                            if (limitSwitch.getState()) {
                                liftMotor.setPower(0);
                                liftMotorOffset = liftMotor.getCurrentPosition();
                                runZeroing = false;
                            } else {
                                liftMotor.setPower(-0.15);
                            }
                        }
                    }

                    // Set the current artifact color.
                    artifacts.set(artifactIndex, new Artifact(getArtifactColor(), currentSlot));

                    // If the artifact is not empty, Move to the next artifact slot
                    if (artifacts.get(artifactIndex).getType() != ArtifactType.EMPTY) {
                        MoveToArtifact(ArtifactType.EMPTY);
                    }
                }

                liftMotor.setPower(pController.getOut());

            } break;
            case Outtake: {

                if (CanMove()) {

                    if (!limitSwitch.getState()) {
                        liftMotor.setPower(-0.15);
                        break;
                    }

                    if (limitSwitch.getState()) {
                        liftMotor.setPower(0);
                        liftMotorOffset = liftMotor.getCurrentPosition();
                    }

                    if (canRotate && limitSwitch.getState()) {
                        canRotate = false;
                        MoveToArtifact(artifactPattern.get(patternIndex));
                    }
                }

                liftMotor.setPower(pController.getOut());
                
            } break;
        }

        rotationServo.setPosition(RotationPosition);
    }

    public void Telemetry() {
        telemetry.addLine("----Artifacts----");
        telemetry.addData("Artifact 1 ", artifacts.get(0).getType());
        telemetry.addData("Artifact 2 ", artifacts.get(1).getType());
        telemetry.addData("Artifact 3 ", artifacts.get(2).getType());
        telemetry.addLine("----Servo Positions----");
        telemetry.addData("Rotation Servo", RotationPosition);
        telemetry.addLine("----State----");
        telemetry.addData("State", state);
        telemetry.addLine("----Timers----");
        telemetry.addData("Rotation Timer", rotationTimer.getSecondsRemaining());
        telemetry.addLine("----Others----");
        telemetry.addData("Current Slot", currentSlot);
        telemetry.addData("Current Artifact", artifactIndex);
        telemetry.addData("Limit Switch", limitSwitch.getState());
        telemetry.addData("Motor Offset", liftMotorOffset);
        telemetry.addData("Motor Position", pController.getCurrentPosition());
        telemetry.addData("Controller Target", pController.getTargetPosition());
        telemetry.addData("Controller Out", pController.getOut());
        telemetry.addData("Motor Position 2", liftMotor.getCurrentPosition());
    }

    // When called, this function sorts a copied array list of artifacts then moves to the next appropriate slot
    private void MoveToArtifact(ArtifactType t) {
        ArrayList<Artifact> a = new ArrayList<>(artifacts);

        sortByType(a, t);

        if (a.get(0).getType() != t) {

            switch (state) {
                case Intake: {
                    state = TransferState.Outtake;
                    MoveToArtifact(artifactPattern.get(patternIndex));
                } break;

                case Outtake: {
                    ArrayList<Artifact> a2 = new ArrayList<>(a);
                    filterByType(a2, ArtifactType.EMPTY);

                    if (a2.isEmpty()) {
                        state = TransferState.Intake;
                        moveToSlot(a.get(0).getSlot());
                        break;
                    } else {
                        moveToSlot(a2.get(0).getSlot());
                    }
                }
            }
        } else {
            moveToSlot(a.get(0).getSlot());
        }
    }

    // Checks if both the rotation and eject timers are finished.
    public boolean CanMove() {
        boolean value = rotationTimer.isFinished();

        if (isEjecting) {
            value = false;
        }

        if (pController.getState() == ControllerStates.RUN_CONTROLLER) {
            value = false;
        }

        return value;
    }

    // Using the three color sensors, it will set the selected artifacts color if the requirements are met;
    private ArtifactType getArtifactColor() {
        ArtifactType value = ArtifactType.EMPTY;

        // Get the total color values from the sensors;
        int totalRed = (sensor1.red() + sensor2.red() + sensor3.red());
        int totalGreen = (sensor1.green() + sensor2.green() + sensor3.green());
        int totalBlue = (sensor1.blue() + sensor2.blue() + sensor3.blue());

        // Check if the values are beyond a certain threshold.
        if (totalRed > RedLimit  && totalGreen > GreenLimit && totalBlue > BlueLimit) {

            // Set the artifact color based on the green to blue ratio;
            if (totalBlue > totalGreen) {
                value = ArtifactType.PURPLE;
            } else if (totalGreen > totalBlue) {
                value = ArtifactType.GREEN;
            }
        }

        return value;
    }

    private void sortByType(ArrayList<Artifact> artifacts, ArtifactType t) {
        artifacts.sort((o1, o2) -> {
            int tmp;

            tmp = (int)Math.signum(o2.isType(t) - o1.isType(t));

            return tmp;
        });
    }

    private void filterByType(ArrayList<Artifact> artifacts, ArtifactType t) {
        ArrayList<Artifact> toRemove = new ArrayList<>();

        for (Artifact a : artifacts) {
            if (a.getType() == t) {
                toRemove.add(a);
            }
        }
        artifacts.removeAll(toRemove);
    }

    private void moveToSlot(int newSlot) {
        switch (newSlot) {
            case 1: RotationPosition = RotationSlot_1_Intake; break;
            case 2: RotationPosition = RotationSlot_2_Intake; break;
            case 3: RotationPosition = RotationSlot_3_Intake; break;
        }
        findMoveTime(currentSlot, newSlot);
        currentSlot = newSlot;
        artifactIndex = currentSlot - 1;
    }

    private void findMoveTime(int currentSlot, int newSlot) {
        switch (currentSlot) {
            case 1: {

                switch (newSlot) {
                    case 2: rotationTimer.setMillisecondTimer(500); break;
                    case 3: rotationTimer.setMillisecondTimer(1000); break;
                }
                break;
            }
            case 2: {
                rotationTimer.setMillisecondTimer(500);
                break;
            }
            case 3: {

                switch (newSlot) {
                    case 2: rotationTimer.setMillisecondTimer(500); break;
                    case 1: rotationTimer.setMillisecondTimer(1000); break;
                }
                break;
            }
        }
    }
}
