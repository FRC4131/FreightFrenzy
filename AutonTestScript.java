package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandparser.CommandParser;
import org.firstinspires.ftc.teamcode.commandparser.FileCommandParser;
import org.firstinspires.ftc.teamcode.robot.ArmBot;
import org.firstinspires.ftc.teamcode.robot.BareBonesBot;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SoftwareBot;
import org.firstinspires.ftc.teamcode.subsystem.tablespinner.SpinnerDirection;

import java.io.FileNotFoundException;
import java.io.IOException;

@Autonomous(name="AutonTestScript", group="")
//@Disabled

public class AutonTestScript extends LinearOpMode {

    @Override
    public void runOpMode() {
//        try {
            //Robot myRobot = new BareBonesBot(telemetry, hardwareMap);
            //Robot myRobot = new SoftwareBot(telemetry, hardwareMap);
            Robot myRobot = new ArmBot(telemetry, hardwareMap);

            /*
            String inputFileName = "/sdcard/tmp/RedTurnTable.txt";
            CommandParser myCommandParser = new FileCommandParser(myRobot, inputFileName, telemetry);
*/
            waitForStart();
/*
            while(myCommandParser.update())
            {
                sleep(1000);
            }
            */
            //sleep(1000);
        myRobot.moveArmPosition(2);
        myRobot.turnArmSpinnerTimed(5, 1, 0.18);

        while (opModeIsActive()) {

                //myRobot.turnSpinnerTimed(3, 0.8, SpinnerDirection.FORWARD.getVal());
            //myRobot.moveStraightInches(-6, 0.1);
            //myRobot.moveSidewaysInches(6,0.3);
            //myRobot.turnAngleDegrees(-45,0.3);

            //sleep(1000);
            }

//        } catch (IOException e) {
//            e.printStackTrace();
//        }
    }
}

