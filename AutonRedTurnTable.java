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
            //Robot myRobot = new BareBonesBot(telemetry, hardwareMap);
            //Robot myRobot = new SoftwareBot(telemetry, hardwareMap);
            Robot myRobot = new ArmBot(telemetry, hardwareMap);

            waitForStart();

            int barCodeTier = myRobot.ScanBarCode();

//            myRobot.moveCollectorArm(1,1.0);
//            sleep(2000);
//            myRobot.moveCollectorArm(0, 0.4);

            double movePower = 0.6;
            myRobot.moveStraightInches(5, movePower);
            myRobot.turnAngleDegrees(185, movePower);
            myRobot.moveSidewaysInches(-31, movePower);
            myRobot.turnSpinner(0.8, 1);
            sleep(3000);
            myRobot.turnSpinner(0, 1);
            myRobot.moveStraightInches(-54, movePower);
            myRobot.moveArmPosition(barCodeTier);
            myRobot.moveSidewaysInches(50, movePower);
            myRobot.turnArmSpinner(1, 0.6);
            sleep(2000);
            myRobot.turnArmSpinner(1, 0.0);
            myRobot.moveStraightInches(-2, movePower);
            myRobot.moveSidewaysInches(24, movePower);
            myRobot.moveStraightInches(31, movePower);
            myRobot.turnAngleDegrees(98, movePower);
            myRobot.moveStraightInches(55, 1.0);
            myRobot.moveArmPosition(-1);
            myRobot.turnAngleDegrees(-95, movePower);
            myRobot.turnArmSpinner(-1,0.6);
            myRobot.moveStraightInches(9,movePower);
            sleep(2000);
            myRobot.turnArmSpinner(-1,0.0);
            myRobot.moveStraightInches(-5,movePower);
            myRobot.turnAngleDegrees(180,movePower);
    }
}

