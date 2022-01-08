package org.firstinspires.ftc.teamcode.robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Bare Bones robot implementation.
    Does nothing interesting aside from print out messages,
    but at a minimum satisfies our Robot interface */
public class BareBonesBot implements Robot{

    Telemetry telemetry;
    HardwareMap hardwareMap;

    public BareBonesBot(Telemetry inputTelemetry, HardwareMap inputHardwareMap){
        this.telemetry = inputTelemetry;
        this.hardwareMap = inputHardwareMap;
    }

    @Override
    public void moveStraightInches(double distanceInches, double power){
        telemetry.addData("Command: ","MoveStraight");
        telemetry.addData("Distance: ", distanceInches);
        telemetry.addData("Power: ", power);
        
        telemetry.update();
    }

    @Override
    public void moveSidewaysInches(double distanceInches, double power){
        telemetry.addData("Command: ","MoveSideways");
        telemetry.addData("Distance: ", distanceInches);
        telemetry.addData("Power: ", power);
        telemetry.update();
    }

    @Override
    public void turnAngleDegrees(double angleDegrees, double power){
        telemetry.addData("Command: ","TurnAngle");
        telemetry.addData("Distance: ", angleDegrees);
        telemetry.addData("Power: ", power);
        telemetry.update();
    }

    @Override
    public void moveArmPosition(int position){
        telemetry.addData("Command: ","MoveArmPosition");
        telemetry.addData("Position: ", position);
        telemetry.update();
    }

    @Override
    public void turnArmSpinnerTimed(double timeSeconds, int direction, double power) {
        telemetry.addData("Command: ", "TimedTurnArmSpinner");
        telemetry.addData("Time: ", timeSeconds);
        telemetry.addData("Power: ", power);
        telemetry.update();
    }

    @Override
    public void turnArmSpinner(int direction, double power) {
        telemetry.addData("Command: ", "TurnArmSpinner");
        telemetry.addData("Power: ", power);
        telemetry.update();
    }

    @Override
    public void turnSpinnerTimed(double timeSeconds, double power, int direction){
        telemetry.addData("Command: ","TurnSpinnerTimed");
        telemetry.addData("Time: ", timeSeconds);
        telemetry.addData("Power: ", power);
        telemetry.addData("Direction: ", direction);
        telemetry.update();
    }

    @Override
    public void turnSpinner(double power, int direction) {
        telemetry.addData("Command: ","TurnSpinner");
        telemetry.addData("Power: ", power);
        telemetry.addData("Direction: ", direction);
        telemetry.update();
    }

    @Override
    public void moveCollectorArm(int position, double power) {

    }

    @Override
    public int ScanBarCode(){
        telemetry.addData("Command: ", "ScanBarCode");
        telemetry.update();
        return 0;
    }

    @Override
    public void stopCameraStream() {

    }

}
