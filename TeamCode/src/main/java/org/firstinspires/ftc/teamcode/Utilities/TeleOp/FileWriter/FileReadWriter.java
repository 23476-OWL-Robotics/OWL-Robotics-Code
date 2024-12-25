package org.firstinspires.ftc.teamcode.Utilities.TeleOp.FileWriter;

import java.io.*;

public class FileReadWriter {

    FileWriter fileWriter;
    FileReader fileReader;

    BufferedWriter writer;
    BufferedReader reader;

    File file;
    String fileName = "./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Utilities/TeleOp/FileWriter/numbers.txt";

    int[] write;
    int[] read;

    public FileReadWriter() {
        try {
            file = new File(fileName);

            fileWriter = new FileWriter(file);
            writer = new BufferedWriter(fileWriter);

            fileReader = new FileReader(file);
            reader = new BufferedReader(fileReader);

            write = new int[8];
            read = new int[8];
        } catch (IOException e) {
            // Nothing
        }
    }

    public void writeFile(double armEncoderPosition,
                          double intakeEncoderPosition,
                          double leftAssentEncoderPosition,
                          double rightAssentEncoderPosition,
                          double robotPoseX,
                          double robotPoseY,
                          double robotHeading) {

        try {
            write[1] = Integer.parseInt(String.valueOf(armEncoderPosition));
            write[2] = Integer.parseInt(String.valueOf(intakeEncoderPosition));
            write[3] = Integer.parseInt(String.valueOf(leftAssentEncoderPosition));
            write[4] = Integer.parseInt(String.valueOf(rightAssentEncoderPosition));
            write[5] = Integer.parseInt(String.valueOf(robotPoseX));
            write[6] = Integer.parseInt(String.valueOf(robotPoseY));
            write[7] = Integer.parseInt(String.valueOf(robotHeading));

            for (int i = 1; i < 8; i++) {
                writer.write(write[i]);
                writer.newLine();
            }
            writer.flush();
        } catch (IOException e) {
            // Nothing
        }
    }

    public void readFile() {

        try {
            read[1] = Integer.parseInt(reader.readLine());
            read[2] = Integer.parseInt(reader.readLine());
            read[3] = Integer.parseInt(reader.readLine());
            read[4] = Integer.parseInt(reader.readLine());
            read[5] = Integer.parseInt(reader.readLine());
            read[6] = Integer.parseInt(reader.readLine());
            read[7] = Integer.parseInt(reader.readLine());

            reader.close();
        } catch (IOException e) {
            // Nothing
        }
    }
}
