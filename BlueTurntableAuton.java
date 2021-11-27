/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "blue_turNtable_auton")
public class BlueTurntableAuton extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor spinner = null;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor starMotor = null;
    private DcMotor arm = null;
    private DcMotor arm2 = null;
    private Servo clamp = null;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double startAngle;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AYARiS3/////AAABmYX7HlUeQEZ4lwV/YqVyX+4mirm35X2Rjl3vmRHtPxu4QcxIgNG7qzCxgvolucJdpwjVlaMrLWorwZM89pGOupCsKqEC0i9xFqPd93fhcoVF/SKAcORXdWoJ9MZjfHlUKVVO4d54A8oN7BxizDOXgN91Ys+cMt7cwGY2ArtiwGThv96Q/lelQdCaRcaPBRydciy5ytDAyqPN8MhAz5Etzk3+iG4WdqUEXgUzgh022udBREGHuRH1FwANXh8aL47AvvlVmjLwfYcxXwuDj94PrO60z6xzYQrMU1yZ5OJOOeFkz4Lqdp3fr8tA6cY3BEJMyrra0CRwsV3XN5iG0bHSXOWSGMCPXAkrSDp0CBPpio5/";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {


//        leftBase = hardwareMap.get(Servo.class, "leftBase");
//        rightBase = hardwareMap.get(Servo.class, "rightBase");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        starMotor = hardwareMap.get(DcMotor.class, "SM");
        arm = hardwareMap.get(DcMotor.class, "ARM");
        arm2 = hardwareMap.get(DcMotor.class, "ARM2");
        clamp = hardwareMap.get(Servo.class, "CLAMP");

        spinner.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initVuforia();
        //initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //int duckPosition;
        //duckPosition = duckBarcode();
        //telemetry.addData("duckPosition", duckPosition);
        telemetry.update();

        sideways(12, 0.7);
        rotateToAngle(-50, 0.8);
        timedSpin(-0.75, 4);
        rotateToAngle(49, 0.8);
        forward(28, 0.8);
        outTake(320,3);
        //rotateToAngle(55, 0.8);
        //forward(10, 0.8);
        rotateToAngle(-87, 0.5);
        //forward(-1, 0.3);
        forward(30, 0.8);
        rotateToAngle(0, 0.8);
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
        while (Math.abs(angleDifference) > 0.5 && opModeIsActive()) {
            double rotation = Range.clip(angleDifference * 0.03 * power, -1, 1);
            telemetry.addData("angleDifference", angleDifference);
            telemetry.addData("power", rotation);
            telemetry.update();
            backLeft.setPower(rotation);
            backRight.setPower(-rotation);
            frontLeft.setPower(rotation);
            frontRight.setPower(-rotation);

            angleDifference = offsetAngle(targetAngle);
            idle();
        }

    }
    private boolean atTarget()
    {
        boolean isAtTarget = false;
        if ((frontLeft.getCurrentPosition() >= frontLeft.getTargetPosition()) && (frontRight.getCurrentPosition() >= frontRight.getTargetPosition()))
        {
            isAtTarget = true;
        }
        return isAtTarget;
    }

    public void sideways(double inches, double power) {
        inches *= Math.sqrt(2);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + inchesToTicks(inches));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - inchesToTicks(inches));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - inchesToTicks(inches));
        backRight.setTargetPosition(backRight.getCurrentPosition() + inchesToTicks(inches));

        runToPosition(power);
    }

    public void forward(double inches, double power) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + inchesToTicks(inches));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + inchesToTicks(inches));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + inchesToTicks(inches));
        backRight.setTargetPosition(backRight.getCurrentPosition() + inchesToTicks(inches));

        runToPosition(power);
    }

    public void runToPosition(double power) {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (!atTarget() && opModeIsActive()) {
            idle();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private int inchesToTicks(double inches) {
        return (int) (inches / (4 * Math.PI) * 1440);
    }

    public void cartesianDrive(double forward, double sideways, double rotate) {
        double FLPower = Range.clip(forward + sideways + rotate, -1, 1);
        double FRPower = Range.clip(forward - sideways - rotate, -1, 1);
        double BLPower = Range.clip(forward - sideways + rotate, -1, 1);
        double BRPower = Range.clip(forward + sideways - rotate, -1, 1);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(FLPower);
        frontRight.setPower(FRPower);
        backLeft.setPower(BLPower);
        backRight.setPower(BRPower);
    }
    public void timedSpin(double power, double runtime){
        ElapsedTime spinnerTime = new ElapsedTime();
        spinnerTime.reset();
        while (spinnerTime.seconds() < runtime){
            spinner.setPower(power);
        }
        spinner.setPower(0);
    }
    public void outTake(int pos, double runtime){
        ElapsedTime armTime = new ElapsedTime();
        armTime.reset();
        while (armTime.seconds() < runtime) {
            arm.setTargetPosition(pos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            starMotor.setPower(-0.18);
        }
        starMotor.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(0);
    }
/*
    public int duckBarcode(){
        int duckbarcode = 0;
            while (duckbarcode == 0)
            {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        telemetry.update();

                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if(recognition.getLabel() == "Duck")
                            {
                                duckbarcode = 1;
                            }
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.update();
                        }
                        // step through the list of recognitions and display boundary info.
                    }
                }
            }
        return duckbarcode;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
    /*
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }*/

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    /*private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }*/

}
