package org.firstinspires.ftc.teamcode.gamepadhandler;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class LogitechGamePadHandler implements GamePadHandler{
    Robot robot;
    Gamepad gamepad;

    public LogitechGamePadHandler(Robot robot, Gamepad gamepad){
        this.robot = robot;
        this.gamepad = gamepad;
    }

    @Override
    public void update(){
        //Here I'm going to put my massive list of if statements for the gamepad->robot controls
    }
}
