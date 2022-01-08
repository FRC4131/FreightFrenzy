package org.firstinspires.ftc.teamcode.subsystem.cappingArm;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MagneticCappingArm implements CappingArm{
    DcMotor arm2Motor;

    public MagneticCappingArm(DcMotor inputCapMotor) {
        this.arm2Motor = inputCapMotor;

    }
    @Override
    public void moveColectorArm(int position, double power){
        if(position == 0) {
            arm2Motor.setTargetPosition(0);
        } else if (position == 1){
            arm2Motor.setTargetPosition(500);
        } else {
            arm2Motor.setTargetPosition(500);
        }
        this.arm2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Motor.setPower(power);
        while(arm2Motor.isBusy()){}
        arm2Motor.setPower(0.0);
    }
}
