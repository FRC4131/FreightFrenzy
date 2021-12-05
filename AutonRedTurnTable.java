package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandparser.CommandParser;
import org.firstinspires.ftc.teamcode.commandparser.FileCommandParser;
import org.firstinspires.ftc.teamcode.robot.ArmBot;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.io.IOException;

@Autonomous(name="AutonRedTurnTable", group="")
//@Disabled

public class AutonRedTurnTable extends LinearOpMode {

    @Override
    public void runOpMode() {
        try {
            //Robot myRobot = new BareBonesBot(telemetry, hardwareMap);
            //Robot myRobot = new SoftwareBot(telemetry, hardwareMap);
            Robot myRobot = new ArmBot(telemetry, hardwareMap);


            String inputFileName = "/sdcard/tmp/RedTurnTable.txt";
            CommandParser myCommandParser = new FileCommandParser(myRobot, inputFileName, telemetry);

            waitForStart();

            while(myCommandParser.update()) {}

//        while (opModeIsActive()) {
            //sleep(1000);
//            }

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}

