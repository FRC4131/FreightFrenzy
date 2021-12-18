package org.firstinspires.ftc.teamcode.subsystem.tablespinner;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SingleMotorTableSpinner implements TableSpinner{
    DcMotor tableSpinnerMotor;

    public SingleMotorTableSpinner(DcMotor inputSpinnerMotor){
        this.tableSpinnerMotor = inputSpinnerMotor;
    }

    @Override
    public void turnSpinnerTimed(double timeSeconds, double power, int direction) {
        double spinnerPower = power * direction;

        ElapsedTime spinnerTime = new ElapsedTime();
        spinnerTime.reset();

        this.tableSpinnerMotor.setPower(spinnerPower);
        while (spinnerTime.seconds() < timeSeconds){
        }
        this.tableSpinnerMotor.setPower(0);
    }

    @Override
    public void turnSpinner(double power, int direction) {
        double spinnerPower = power * direction;
        this.tableSpinnerMotor.setPower(spinnerPower);
    }
}
