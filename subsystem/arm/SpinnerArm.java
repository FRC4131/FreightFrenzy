package org.firstinspires.ftc.teamcode.subsystem.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SpinnerArm implements Arm {
    DcMotor armMotor;
    DcMotor starMotor;

    public SpinnerArm(DcMotor inputArmMotor, DcMotor inputStarMotor){
        this.armMotor = inputArmMotor;
        this.starMotor = inputStarMotor;
        //this.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void moveToTier(int tier, double power) {
        if(tier == 0) {
            armMotor.setTargetPosition(320);
        } else if (tier == 1){
            armMotor.setTargetPosition(560);
        } else {
            armMotor.setTargetPosition(830);
        }
        this.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
        while(armMotor.isBusy()){
        }
        //this.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotor.setPower(0);
    }

    @Override
    public void cargoSpinTimed(int direction, double time, double power) {
        ElapsedTime starTime = new ElapsedTime();
        starTime.reset();
        while (starTime.seconds() <= time) {
            starMotor.setPower(direction * power);
        }
        starMotor.setPower(0);
    }

    @Override
    public void cargoSpin(int direction, double power) {
        starMotor.setPower(direction * power);
    }
}
