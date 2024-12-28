package org.firstinspires.ftc.teamcode.Utilities.TeleOp.FileWriter;

import android.annotation.SuppressLint;
import android.os.Environment;

import java.io.*;
import java.util.Arrays;

public class FileReadWriter {

    FileWriter fileWriter;
    FileReader fileReader;

    BufferedWriter writer;
    BufferedReader reader;

    File file;
    String fileName = Environment.getExternalStorageDirectory().getPath() + "/FIRST/data/numbers.txt";

    public double[] write;
    public double[] read;

    public FileReadWriter() throws IOException {
        write = new double[8];
        read = new double[8];
    }

    public void writeFile(double armEncoderPosition,
                          double intakeEncoderPosition,
                          double leftAssentEncoderPosition,
                          double rightAssentEncoderPosition,
                          double robotPoseX,
                          double robotPoseY,
                          double robotHeading) throws IOException{

        file = new File(fileName);
        fileWriter = new FileWriter(file);
        writer = new BufferedWriter(fileWriter);

        write[1] = armEncoderPosition;
        write[2] = intakeEncoderPosition;
        write[3] = leftAssentEncoderPosition;
        write[4] = rightAssentEncoderPosition;
        write[5] = robotPoseX;
        write[6] = robotPoseY;
        write[7] = robotHeading;

        for (int i = 1; i <= 7; i++) {
            writer.write(Double.toString(write[i]));
            writer.newLine();
        }
        writer.close();
    }

    public void readFile() throws IOException {

        file = new File(fileName);
        fileReader = new FileReader(file);
        reader = new BufferedReader(fileReader);

        read[1] = Double.parseDouble(reader.readLine());
        read[2] = Double.parseDouble(reader.readLine());
        read[3] = Double.parseDouble(reader.readLine());
        read[4] = Double.parseDouble(reader.readLine());
        read[5] = Double.parseDouble(reader.readLine());
        read[6] = Double.parseDouble(reader.readLine());
        read[7] = Double.parseDouble(reader.readLine());

        reader.close();
    }

    public void clearFile() throws IOException {

        writer = new BufferedWriter(fileWriter);

        write[1] = 0;
        write[2] = 0;
        write[3] = 0;
        write[4] = 0;
        write[5] = 0;
        write[6] = 0;
        write[7] = 0;

        for (int i = 1; i <= 7; i++) {
            writer.write(Double.toString(write[i]));
            writer.newLine();
        }
        writer.close();
    }
}
