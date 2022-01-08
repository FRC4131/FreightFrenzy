package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandparser.CommandParser;
import org.firstinspires.ftc.teamcode.commandparser.FileCommandParser;
import org.firstinspires.ftc.teamcode.robot.ArmBot;
import org.firstinspires.ftc.teamcode.robot.BareBonesBot;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.io.IOException;

@Autonomous(name="AutonBlueTurnTable", group="")
//@Disabled

public class AutonBlueTurnTable extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
            //Robot myRobot = new BareBonesBot(telemetry, hardwareMap);
            //Robot myRobot = new SoftwareBot(telemetry, hardwareMap);
            Robot myRobot = new ArmBot(telemetry, hardwareMap);

            waitForStart();

            int barCodeTier = myRobot.ScanBarCode();
            myRobot.stopCameraStream();
            double movePower = 0.75;
            //myRobot.moveCollectorArm(1,0.4);
//            sleep(10000);
            myRobot.moveStraightInches(4, movePower);
            myRobot.turnAngleDegrees(-90, movePower);
            myRobot.moveStraightInches(13.5, 0.6);

            myRobot.turnSpinner(0.8, -1);
            sleep(3000);
            myRobot.turnSpinner(0, -1);
            myRobot.moveSidewaysInches(60, movePower);
            myRobot.moveArmPosition(barCodeTier);
            myRobot.turnAngleDegrees(-100, movePower);
            myRobot.moveSidewaysInches(-45, movePower);
            myRobot.turnArmSpinner(1, 0.36);
            sleep(2000);
            myRobot.turnArmSpinner(1, 0.0);
            myRobot.moveSidewaysInches(-23, movePower);
            myRobot.moveStraightInches(35, movePower);
            myRobot.turnAngleDegrees(-90, movePower);
            myRobot.moveStraightInches(53, 1.0);
            myRobot.turnAngleDegrees(-90, movePower);
    }
}

