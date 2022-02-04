package org.firstinspires.ftc.teamcode;
//package com.arcrobotics.ftclib.gamepad;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

@TeleOp(name="Field Centric Mecanum Red")

public class FieldCentricMecanumRed extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor starMotor = null;
    private DcMotor arm = null;
    private DcMotor arm2 = null;
    private DcMotor spinner = null;
    private Servo clamp = null;
    private Servo cappingHand = null;
    private Servo linear = null;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double startAngle;

    double savedAngle_offset = 0.0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        FileReader file = null;
        BufferedReader bufferedReader;
        String angleString = "0.0";

        try {
            file = new FileReader("/sdcard/tmp/SavedHeading.txt");
            bufferedReader = new BufferedReader(file);
            angleString = bufferedReader.readLine();
            bufferedReader.close();
            File myfile = new File("/sdcard/tmp/SavedHeading.txt");
            myfile.delete();

        } catch (FileNotFoundException e) {
        } catch (IOException e) {
        }

        savedAngle_offset = Double.parseDouble(angleString);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        // parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft  = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        starMotor = hardwareMap.get(DcMotor.class, "SM");
        arm = hardwareMap.get(DcMotor.class, "ARM");
        arm2 = hardwareMap.get(DcMotor.class, "ARM2");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        cappingHand = hardwareMap.get(Servo.class, "CAPPINGHAND");
        clamp = hardwareMap.get(Servo.class, "CLAMP");
        linear = hardwareMap.get(Servo.class, "LINEAR");
        imu.initialize(parameters);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        clamp.setDirection(Servo.Direction.FORWARD);
        linear.setDirection(Servo.Direction.FORWARD);
        cappingHand.setDirection(Servo.Direction.FORWARD);
        arm2.setDirection(DcMotor.Direction.FORWARD);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Setting the DcMotors to run using the encoders engages the DcMotor's built-in
        // PID controller when setting motor angular speeds
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        startAngle = angles.firstAngle;
    }
    public void starAngle(int pos){

        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double backLeftPower;
        double backRightPower;
        double frontLeftPower;
        double frontRightPower;
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.
        // send the info back to driver station using telemetry function.
        // if the digital channel returns true it's HIGH and the button is unpressed.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rotation = -gamepad1.right_stick_x;

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double c = angles.firstAngle - startAngle + savedAngle_offset;
        double d = Math.toRadians(-c);
        double driveSpeed;
        if(gamepad1.right_trigger == 1.0){
            driveSpeed = .4;
        } else {
            driveSpeed = 1.0; //doubled from 0.45 (using 20:1 gear boxes) for new 40:1 gear boxes
        }

        double forward = -x * Math.sin(d) + y * Math.cos(d);
        double sideways = x * Math.cos(d) + y * Math.sin(d);
        frontLeftPower =  Range.clip(forward - sideways - rotation, -driveSpeed, driveSpeed);
        frontRightPower = Range.clip(forward + sideways + rotation, -driveSpeed, driveSpeed);
        backLeftPower =   Range.clip(forward + sideways - rotation, -driveSpeed, driveSpeed);
        backRightPower =  Range.clip(forward - sideways + rotation, -driveSpeed, driveSpeed);

        //Activates star motor to intake on the arm if the x button is pressed down on gamepad 2
        //linear actuator simultaneously slightly opens the cup to enable intake
        //Activates star motor to output on the arm if the Y button is pressed on gamepad 2
        //linear actuator simultaneously opens the cup wide to enable output
        //when no button is pressed, cup is in slightly closed position
        if(gamepad2.x) {
            starMotor.setPower(1.0);
            linear.setPosition(0.25);
        } else if (gamepad2.y) {
            starMotor.setPower(-0.40);
            linear.setPosition(0.4);
        } else {
            starMotor.setPower(0);
            linear.setPosition(0.22);
        }
        if(gamepad2.left_stick_button){ //push really hard on the left stick
            starAngle(400);
        }

        //left bumper puts Arm2 in low position to grab
        //holding left trigger puts Arm2 in capping position
        //holding right bumper puts Arm back in initialization position
        if(gamepad2.left_bumper){
            arm2.setTargetPosition(545);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setPower(0.8);
            cappingHand.setPosition(0.0);
        } else if(gamepad2.right_bumper) {
            arm2.setTargetPosition(0);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setPower(-1);
            cappingHand.setPosition(0);
        } else if (gamepad2.left_trigger == 1.0) {
            arm2.setTargetPosition(335);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setPower(1.0);
            cappingHand.setPosition(0.52);
        } else if (gamepad2.b){
          arm2.setTargetPosition(350);
          arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          arm2.setPower(1.0);
          cappingHand.setPosition(0.52);
        } else{
            arm2.setPower(0);
        }

        if(gamepad2.right_trigger == 1.0){
            clamp.setPosition(0.7);

        }
        else{
            clamp.setPosition(.3);
        }
        if(gamepad2.dpad_up) {

            starAngle(830);

        }
        if(gamepad2.dpad_right) {

            starAngle(560);

        }
        if(gamepad2.dpad_down) {

            starAngle(310);

        }
        if(gamepad2.dpad_left) {

            starAngle(0);

        }

        if(gamepad2.a) {
            spinner.setPower(-1);
        } else {
            spinner.setPower(0);
        }

        //rotates robot in the direction the Dpad is pressed - maintains field centric

        if(gamepad1.dpad_down){
            rotateToAngle(180, 1);
        }
        if(gamepad1.dpad_up){
            rotateToAngle(0, 1);
        }
        if(gamepad1.dpad_left){
            rotateToAngle(90, 1);
        }
        if(gamepad1.dpad_right){
            rotateToAngle(270, 1);
        }

        // Send calculated power to wheels
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);


        telemetry.addData("Heading", angles.firstAngle);
        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
    public void timedRotate(double runtime, double speed){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(timer.seconds() < runtime){
            backLeft.setPower(speed);
            frontLeft.setPower(speed);
            backRight.setPower(-speed);
            frontRight.setPower(-speed);
        }
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
    public double offsetAngle(double targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();

        double currentAngle = angles.firstAngle - startAngle;

        double angleDifference = currentAngle - targetAngle;

        while (angleDifference < -180) {
            angleDifference += 360;
        }
        while (angleDifference > 180) {
            angleDifference -= 360;
        }

        return angleDifference;

    }
    public void rotateToAngle(double targetAngle, double power) {
        double angleDifference = offsetAngle(targetAngle);
        while (Math.abs(angleDifference) > 0.5) {
            double rotation = Range.clip(angleDifference * 0.03 * power, -1, 1);
            telemetry.addData("angleDifference", angleDifference);
            telemetry.addData("power", rotation);
            telemetry.update();
            backLeft.setPower(rotation);
            backRight.setPower(-rotation);
            frontLeft.setPower(rotation);
            frontRight.setPower(-rotation);

            angleDifference = offsetAngle(targetAngle);
        }

    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}