package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandparser.CommandParser;
import org.firstinspires.ftc.teamcode.commandparser.FileCommandParser;
import org.firstinspires.ftc.teamcode.robot.ArmBot;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.io.IOException;

@Autonomous(name="AutonBlueTurnTable", group="")
//@Disabled

public class AutonBlueTurnTable extends LinearOpMode {

    @Override
    public void runOpMode() {
            //Robot myRobot = new BareBonesBot(telemetry, hardwareMap);
            //Robot myRobot = new SoftwareBot(telemetry, hardwareMap);
            Robot myRobot = new ArmBot(telemetry, hardwareMap);

            waitForStart();

            int barCodeTier = myRobot.ScanBarCode();
            myRobot.stopCameraStream();
            double movePower = 0.4;


            myRobot.moveStraightInches(4, movePower);
            myRobot.turnAngleDegrees(-90, movePower);
            myRobot.moveStraightInches(13, 0.2);
            myRobot.turnSpinnerTimed(3, 0.8, -1);
            myRobot.moveSidewaysInches(56, movePower);
            myRobot.moveArmPosition(barCodeTier);
            myRobot.turnAngleDegrees(-100,movePower);
            myRobot.moveSidewaysInches(-43, movePower);
            myRobot.turnArmSpinnerTimed(2,1, 0.36);
            myRobot.moveSidewaysInches(-25, movePower);
            myRobot.moveStraightInches(31, movePower);
            myRobot.turnAngleDegrees(-90, movePower);
            myRobot.moveStraightInches(53, 1.0);
            myRobot.turnAngleDegrees(-90, movePower);

//            myRobot.moveStraightInches(-51, movePower);
//            myRobot.moveArmPosition(barCodeTier);
//
//
//            myRobot.moveStraightInches(-2, movePower);
//            myRobot.moveSidewaysInches(24, movePower);
//            myRobot.moveStraightInches(31, movePower);
//            myRobot.turnAngleDegrees(90, movePower);
//            myRobot.moveStraightInches(55, 1.0);
//            myRobot.turnAngleDegrees(90, movePower);

//        while (opModeIsActive()) {
//            }

//        } catch (IOException e) {
//            e.printStackTrace();
//        }
    }
}

