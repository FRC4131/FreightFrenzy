package org.firstinspires.ftc.teamcode.subsystem.cappingArm;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MagneticCappingArm implements CappingArm{
    DcMotor arm2Motor;
    Servo  cappingHand;

    public MagneticCappingArm(DcMotor inputCapMotor, Servo inputCappingHand) {
        this.arm2Motor = inputCapMotor;
        this.cappingHand = inputCappingHand;

    }
    @Override
    public void moveColectorArm(int position, double power){
        if(position == 0) {
            arm2Motor.setTargetPosition(0);
            cappingHand.setPosition(.08);

        } else if (position == 1){
            arm2Motor.setTargetPosition(500);
            cappingHand.setPosition(0.0);
        } else {
            arm2Motor.setTargetPosition(500);
            cappingHand.setPosition(0.0);
        }
        this.arm2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Motor.setPower(power);
        while(arm2Motor.isBusy()){}
        arm2Motor.setPower(0.0);
    }
}
