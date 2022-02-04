package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.ArmBot;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="AutonRedCappingTurnTable", group="")
//@Disabled

public class AutonRedCappingTurnTable extends LinearOpMode {

    @Override
    public void runOpMode() {
            //Robot myRobot = new BareBonesBot(telemetry, hardwareMap);
            //Robot myRobot = new SoftwareBot(telemetry, hardwareMap);
            Robot myRobot = new ArmBot(telemetry, hardwareMap);

            waitForStart();

            int barCodeTier = myRobot.ScanBarCode();
            double movePower = 0.6;

//            myRobot.moveCollectorArm(1,1.0);
//            sleep(2000);
//            myRobot.moveCollectorArm(0, 0.4);


//            myRobot.moveStraightInches(5, movePower);
//            myRobot.turnAngleDegrees(185, movePower);
//            myRobot.moveSidewaysInches(-32, movePower);
            if (barCodeTier==0){
                    myRobot.moveSidewaysInches(14,movePower);
                    myRobot.moveStraightInches(4,movePower);
                    myRobot.moveCollectorArm(2,0.6);
                    myRobot.moveStraightInches(7,movePower);
                    myRobot.moveCollectorArm(0,0.4);
                    myRobot.moveStraightInches(-7,movePower);
                    myRobot.turnAngleDegrees(185, movePower);
                    myRobot.moveSidewaysInches(-19.5, 0.6);

            }
            else if (barCodeTier==1){
                    myRobot.moveSidewaysInches(4,movePower);
                    myRobot.moveStraightInches(4,movePower);
                    myRobot.moveCollectorArm(2,0.6);
                    myRobot.moveStraightInches(7,movePower);
                    myRobot.moveCollectorArm(0,0.4);
                    myRobot.moveStraightInches(-5,movePower);
                    myRobot.turnAngleDegrees(188, movePower);
                    myRobot.moveSidewaysInches(-27.8, 0.6);
            }
            else {
                    myRobot.moveSidewaysInches(-7,movePower);
                    myRobot.moveStraightInches(4,movePower);
                    myRobot.moveCollectorArm(2,0.6);
                    myRobot.moveStraightInches(7,movePower);
                    myRobot.moveCollectorArm(0,0.4);
                    myRobot.moveStraightInches(-7,movePower);
                    myRobot.turnAngleDegrees(184, movePower);
                    myRobot.moveSidewaysInches(-37, 0.6);
            }
            myRobot.turnSpinner(0.8, 1);
            sleep(3000);
            myRobot.turnSpinner(0, 1);
            myRobot.moveStraightInches(-54, movePower);
            myRobot.moveArmPosition(barCodeTier);
            myRobot.moveSidewaysInches(50.5, movePower);
            myRobot.turnArmSpinner(1, 0.4);
            sleep(2000);
            myRobot.turnArmSpinner(1, 0.0);
            myRobot.moveStraightInches(-2, movePower);
            myRobot.moveSidewaysInches(23, movePower);
            myRobot.moveStraightInches(33, movePower);
            myRobot.turnAngleDegrees(98, movePower);
            myRobot.moveStraightInches(52, 1.0);
            //myRobot.moveArmPosition(-1);
//            myRobot.turnAngleDegrees(95, movePower);
            //myRobot.turnArmSpinner(-1,0.6);
            //myRobot.moveStraightInches(9,movePower);
            //sleep(2000);
            //myRobot.turnArmSpinner(-1,0.0);
            //myRobot.moveStraightInches(-5,movePower);
            //myRobot.turnAngleDegrees(185,movePower);*/
            myRobot.saveHeading();
    }
}

