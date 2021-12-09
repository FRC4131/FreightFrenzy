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

            int barCodeTier = myRobot.ScanBarCode();

            double movePower = 0.4;
            myRobot.moveStraightInches(2, movePower);
            myRobot.moveSidewaysInches(-18, movePower);
            myRobot.moveStraightInches(15, movePower);
            myRobot.moveArmPosition(barCodeTier);
            myRobot.turnArmSpinnerTimed(2,1, 0.3);
            myRobot.turnAngleDegrees(90, movePower);
            myRobot.moveStraightInches(54, movePower);
            myRobot.turnAngleDegrees(-90, movePower);
            myRobot.moveStraightInches(4, 0.8);
    }
}

