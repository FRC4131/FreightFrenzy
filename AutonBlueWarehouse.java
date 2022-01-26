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
            //Robot myRobot = new BareBonesBot(telemetry, hardwareMap);
            //Robot myRobot = new SoftwareBot(telemetry, hardwareMap);
            Robot myRobot = new ArmBot(telemetry, hardwareMap);

            waitForStart();
            double movePower = 0.8;
            int barCodeTier = myRobot.ScanBarCode();
            myRobot.moveStraightInches(2, movePower);
            myRobot.moveSidewaysInches(-18, movePower);
            myRobot.moveArmPosition(barCodeTier);
            myRobot.moveStraightInches(15, movePower);
            myRobot.turnArmSpinner(1, 0.5);
            sleep(2000);
            myRobot.turnArmSpinner(1, 0.0);
            myRobot.moveStraightInches(-5, movePower);
            myRobot.turnAngleDegrees(90, movePower);
            myRobot.moveStraightInches(54, 1.0);
            //myRobot.turnAngleDegrees(-90, movePower);
            //myRobot.moveStraightInches(6, movePower);
    }
}

