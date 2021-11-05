package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="BasicMecanum", group="")
//@Disabled
public class BasicMecanum extends LinearOpMode {

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Map the DcMotor objects to the Robot Controller motor names (in the Robot config)
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        //A positive rotation of a motor set to "FORWARD" move the motor in the CCW direction.
        //These motor directions were set here as per the location/orientation of the motors
        // as positioned on the robot's frame.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        //Setting the DcMotors to run using the encoders engages the DcMotor's built-in
        // PID controller when setting motor angular speeds
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            double omega_fl = v_x  - v_y  - omega_z;
            double omega_fr = v_x  + v_y  + omega_z;
            double omega_bl = v_x  + v_y  - omega_z;
            double omega_br = v_x  - v_y  + omega_z;

            //The setPower method sets the desired normalized angular speed if RUN_USING_ENCODER
            // is set otherwise it is just setting a desired normalized motor power/duty cycle
            frontLeft.setPower(omega_fl);
            frontRight.setPower(omega_fr);
            backLeft.setPower(omega_bl);
            backRight.setPower(omega_br);

            telemetry.addData("w_FL", omega_fl);
            telemetry.addData("w_FR", omega_fr);
            telemetry.addData("w_BL", omega_bl);
            telemetry.addData("w_BR", omega_br);
            telemetry.update();
        }
    }
}