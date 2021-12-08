package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandparser.CommandParser;
import org.firstinspires.ftc.teamcode.commandparser.FileCommandParser;
import org.firstinspires.ftc.teamcode.robot.ArmBot;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.io.IOException;

@Autonomous(name="AutonBlueWarehouse", group="")
//@Disabled

public class AutonBlueWarehouse extends LinearOpMode {

    @Override
    public void runOpMode() {
//        try {
            //Robot myRobot = new BareBonesBot(telemetry, hardwareMap);
            //Robot myRobot = new SoftwareBot(telemetry, hardwareMap);
            Robot myRobot = new ArmBot(telemetry, hardwareMap);


//            String inputFileName = "/sdcard/tmp/BlueWarehouse.txt";
//            CommandParser myCommandParser = new FileCommandParser(myRobot, inputFileName, telemetry);

            waitForStart();

//            while(myCommandParser.update()) {}
//   SCANBARCODE
//STRAIGHT 2 0.4
//SIDEWAYS -18 0.4
//STRAIGHT 15 0.4
//MOVEARM
//MOVEARMSPINNER 2 1 0.3
//TURNANGLE 90 0.4
//STRAIGHT 54 0.8
//TURNANGLE -90 0.4
//STRAIGHT 6 0.4









//        while (opModeIsActive()) {
            //sleep(1000);
//            }

//        } catch (IOException e) {
//            e.printStackTrace();
//        }
    }
}

