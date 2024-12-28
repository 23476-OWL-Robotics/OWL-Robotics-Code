package org.firstinspires.ftc.teamcode.TeleOp.Other;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.TeleOp.FileWriter.FileReadWriter;

import java.io.File;
import java.io.IOException;

@TeleOp
public class FilePathFinder extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();
        if (opModeIsActive()) {

            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();

            FileReadWriter fileReadWriter;

            try {
                fileReadWriter = new FileReadWriter();

                fileReadWriter.writeFile(
                        1.1,
                        2.2,
                        3.3,
                        4.4,
                        5.5,
                        6.6,
                        7.7
                );

                fileReadWriter.readFile();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

            double timeTaken = elapsedTime.milliseconds();

            while (opModeIsActive()) {
                telemetry.addData("Write 1", fileReadWriter.read[1]);
                telemetry.addData("Write 2", fileReadWriter.read[2]);
                telemetry.addData("Write 3", fileReadWriter.read[3]);
                telemetry.addData("Write 4", fileReadWriter.read[4]);
                telemetry.addData("Write 5", fileReadWriter.read[5]);
                telemetry.addData("Write 6", fileReadWriter.read[6]);
                telemetry.addData("Write 7", fileReadWriter.read[7]);
                telemetry.addLine();
                telemetry.addData("Time Taken", timeTaken);
                telemetry.update();
            }
        }
    }
}
