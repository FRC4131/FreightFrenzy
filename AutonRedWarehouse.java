package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandparser.CommandParser;
import org.firstinspires.ftc.teamcode.commandparser.FileCommandParser;
import org.firstinspires.ftc.teamcode.robot.ArmBot;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.io.IOException;

@Autonomous(name="AutonRedWarehouse", group="")
//@Disabled

public class AutonRedWarehouse extends LinearOpMode {

    @Override
    public void runOpMode() {
            //Robot myRobot = new BareBonesBot(telemetry, hardwareMap);
            //Robot myRobot = new SoftwareBot(telemetry, hardwareMap);
            Robot myRobot = new ArmBot(telemetry, hardwareMap);

            waitForStart();

            int barCodeTier = myRobot.ScanBarCode();
            telemetry.addData("tier ", barCodeTier);
            telemetry.update();

            double movePower = 0.4;
            myRobot.moveStraightInches(2, movePower);
            myRobot.moveSidewaysInches(18, movePower);
            myRobot.moveStraightInches(16, movePower);
            myRobot.moveArmPosition(barCodeTier);
            myRobot.turnArmSpinnerTimed(2,1, 0.5);
            myRobot.moveStraightInches(-5, movePower);
            myRobot.turnAngleDegrees(-90, movePower);
            myRobot.moveStraightInches(60, 0.8);
            myRobot.turnAngleDegrees(90, movePower);
    }
}

