package org.firstinspires.ftc.teamcode.commandparser;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;
import java.util.Arrays;

public class FileCommandParser implements CommandParser{

    Robot robot;
    private BufferedReader reader;
    Telemetry telemetry;

    public FileCommandParser(Robot inputRobot, String inputFileName, Telemetry inputTelemetry) throws FileNotFoundException {
        this.robot = inputRobot;
        this.reader = new BufferedReader(new FileReader(inputFileName));
        this.telemetry = inputTelemetry;
    }

    private void parseLine(String line){
        List<String> args = Arrays.asList(line.split("\\s+"));
        String command = args.get(0);

        switch(command)
        {
            case "#" :
                this.telemetry.addData("Skipped Command: ", args.get(0));
                this.telemetry.update();
                break;
            case "TIMEDSPIN" :
                this.robot.turnSpinnerTimed(
                        Double.parseDouble(args.get(1)),
                        Double.parseDouble(args.get(2)),
                        Integer.parseInt(args.get(3)));
                break;
            case "TURNANGLE" :
                this.robot.turnAngleDegrees(
                        Double.parseDouble(args.get(1)),
                        Double.parseDouble(args.get(2)));
                break;
            case "STRAIGHT" :
                this.robot.moveStraightInches(
                        Double.parseDouble(args.get(1)),
                        Double.parseDouble(args.get(2)));
                break;
            case "SIDEWAYS" :
                this.robot.moveSidewaysInches(
                        Double.parseDouble(args.get(1)),
                        Double.parseDouble(args.get(2)));
                break;
            case "MOVEARM" :
                this.robot.moveArmPosition(Integer.parseInt(args.get(1)));
                break;
            case "MOVEARMSPINNER":
                this.robot.turnArmSpinnerTimed(
                        Double.parseDouble(args.get(1)),
                        Integer.parseInt(args.get(2)),
                        Double.parseDouble(args.get(3))
                );
                break;
            case "SCANBARCODE":
                this.robot.ScanBarCode();
                break;
            default:
                this.telemetry.addData("Command Not Recognized: ", args.get(0));
                this.telemetry.update();
        }
    }

    @Override
    public boolean update() throws IOException {
        String line = this.reader.readLine();
        if (line != null){
            this.parseLine(line);
            return true;
        }
        else {
            return false;
        }
    }
}
