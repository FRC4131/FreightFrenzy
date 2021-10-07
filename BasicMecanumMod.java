package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="BasicMecanumMod", group="")
//@Disabled
public class BasicMecanumMod extends LinearOpMode {

    private DcMotor D1 = null;
    private DcMotor D2 = null;
    private DcMotor D3 = null;
    private DcMotor D4 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        D1 = hardwareMap.get(DcMotor.class, "FL");
        D2 = hardwareMap.get(DcMotor.class, "FR");
        D3 = hardwareMap.get(DcMotor.class, "BL");
        D4 = hardwareMap.get(DcMotor.class, "BR");

        //A positive rotation of a motor set to "FORWARD" move the motor in the CCW direction.
        //These motor directions were set here as per the location/orientation of the motors
        // as positioned on the robot's frame.
        D1.setDirection(DcMotor.Direction.FORWARD);
        D2.setDirection(DcMotor.Direction.REVERSE);
        D3.setDirection(DcMotor.Direction.FORWARD);
        D4.setDirection(DcMotor.Direction.REVERSE);

        //Setting the DcMotors to run using the encoders engages the DcMotor's built-in
        // PID controller when setting motor angular speeds
        D1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        D2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        D3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        D4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            double degree = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) *180/Math.PI + 90;
            if (degree<0) {
                degree += 360;
            }

            double Magnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            // Here, the +x axis is pointing in the forward direction of the robot
            // the +y axis is found using a right-hand rule.
            // As the gamepad has inverted axis and inverted x,y labels, they were adjusted here.
            double v_y = -gamepad1.left_stick_x;
            double v_x = -gamepad1.left_stick_y;
            double omega_z = -gamepad1.right_stick_x;

            //determine wheel speeds using a 4-wheel Mecanum kinematic model
            double omega_1 = v_x  - v_y  - omega_z;
            double omega_2 = v_x  + v_y  + omega_z;
            double omega_3 = v_x  + v_y  - omega_z;
            double omega_4 = v_x  - v_y  + omega_z;

            //The setPower method sets the desired normalized angular speed if RUN_USING_ENCODER
            // is set otherwise it is just setting a desired normalized motor power/duty cycle
            D1.setPower(omega_1);
            D2.setPower(omega_2);
            D3.setPower(omega_3);
            D4.setPower(omega_4);

            telemetry.addData("w1", omega_1);
            telemetry.addData("w2", omega_2);
            telemetry.addData("w3", omega_3);
            telemetry.addData("w4", omega_4);
            telemetry.update();
        }
    }
}