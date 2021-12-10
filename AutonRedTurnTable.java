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
      //  try {
            //Robot myRobot = new BareBonesBot(telemetry, hardwareMap);
            //Robot myRobot = new SoftwareBot(telemetry, hardwareMap);
            Robot myRobot = new ArmBot(telemetry, hardwareMap);


//           String inputFileName = "/sdcard/tmp/RedTurnTable.txt";
//            CommandParser myCommandParser = new FileCommandParser(myRobot, inputFileName, telemetry);

            waitForStart();

            int barCodeTier = myRobot.ScanBarCode();

            double movePower = 0.4;

            myRobot.moveStraightInches(4, movePower);
            myRobot.turnAngleDegrees(180, movePower);
            myRobot.moveSidewaysInches(-29, movePower);
            myRobot.turnSpinnerTimed(3, 0.8, 1);
            myRobot.moveStraightInches(-48, movePower);
            myRobot.moveSidewaysInches(50, movePower);
            myRobot.moveArmPosition(barCodeTier);
            myRobot.turnArmSpinnerTimed(2,1, 0.36);
            myRobot.moveStraightInches(-2, movePower);
            myRobot.moveSidewaysInches(24, movePower);
            myRobot.moveStraightInches(31, movePower);
            myRobot.turnAngleDegrees(90, movePower);
            myRobot.moveStraightInches(55, 1.0);
            myRobot.turnAngleDegrees(90, movePower);

            //robot lined up on far edge away from turn table
        //    while(myCommandParser.update()) {}

//        while (opModeIsActive()) {
            //sleep(1000);
//            }

       // } catch (IOException e) {
         //  e.printStackTrace();
       // }
    }
}

