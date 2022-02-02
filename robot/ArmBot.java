package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystem.arm.SpinnerArm;
import org.firstinspires.ftc.teamcode.subsystem.cappingArm.CappingArm;
import org.firstinspires.ftc.teamcode.subsystem.cappingArm.MagneticCappingArm;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.BareBonesDriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.positiontracker.IMUPositionTracker;
import org.firstinspires.ftc.teamcode.subsystem.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.subsystem.tablespinner.BareBonesTableSpinner;
import org.firstinspires.ftc.teamcode.subsystem.tablespinner.SingleMotorTableSpinner;
import org.firstinspires.ftc.teamcode.subsystem.tablespinner.TableSpinner;
import org.firstinspires.ftc.teamcode.subsystem.visionsystem.BareBonesVisionSystem;
import org.firstinspires.ftc.teamcode.subsystem.visionsystem.OpenCvVisionSystem;
import org.firstinspires.ftc.teamcode.subsystem.visionsystem.VisionSystem;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

/* Software test robot implementation.
    Should have a basic Mecanum drivetrain, a spinner and webcam,
    and should satisfy our Robot interface */

public class ArmBot implements Robot{
    //High level robot subsystems
    DriveTrain driveTrain;
    TableSpinner spinner;
    VisionSystem visionSystem;
    PositionTracker positionTracker;
    Arm arm;
    CappingArm arm2;


    //Robot messaging/debugging and hardware mapping components
    Telemetry telemetry;
    HardwareMap hardwareMap;

    /*LOW LEVEL ROBOT COMPONENTS (i.e. parts that makeup the high-level components (ex: motors, servos, webcam, etc.)*/
    //Robot driveTrain components and parameters
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    static double gearRatio = 40;  //i.e. a gearbox with 40:1 gear ratio
    static double ticksPerMotorRotation = 24;  //i.e. (6 encoder cycles)/(motor revolution) * (4 encoder ticks or events)/(encoder cycle) = (24 ticks)/(motor revolution) -- see Torquenado encoder Specs
    static double driveBaseLengthInches = 12.25;
    static double driveBaseWidthInches = 10.5;
    static double wheelRadiusInches = 1.89; // "GoBuilda" wheel radius: 48 mm -> approx 1.89 inches

    //Table spinner components
    DcMotor tableSpinnerMotor = null;

    //Vision system components (need to set to null?)
    OpenCvWebcam webcam;

    //Position tracker components (for now we only care about heading)
    BNO055IMU imu;

    //Low level components for arm
    DcMotor armMotor;
    DcMotor starMotor;

    //Arm 2 components (for capping)
    DcMotor arm2Motor;
    Servo cappingHand;

    public ArmBot(Telemetry inputTelemetry, HardwareMap inputHardwareMap){
        this.telemetry = inputTelemetry;
        this.hardwareMap = inputHardwareMap;
        this.positionTracker = initializePositionTracker();
        this.driveTrain = initializeDriveTrain();
        this.spinner = initializeTableSpinner();
        this.visionSystem = initializeVisionSystem();
        this.arm = initializeArm();
        this.arm2 = initializeArm2();
    }

    private CappingArm initializeArm2(){
        this.arm2Motor = hardwareMap.get(DcMotor.class, "ARM2");
        this.arm2Motor.setDirection(DcMotor.Direction.FORWARD);
        this.cappingHand = hardwareMap.get(Servo.class, "CAPPINGHAND");
        this.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return new MagneticCappingArm(this.arm2Motor, this.cappingHand);
    }

    private DriveTrain initializeDriveTrain(){
        //Map the hardware motors to our DCMotor objects
        this.leftFrontMotor = hardwareMap.get(DcMotor.class, "FL");
        this.rightFrontMotor = hardwareMap.get(DcMotor.class, "FR");
        this.leftBackMotor = hardwareMap.get(DcMotor.class, "BL");
        this.rightBackMotor = hardwareMap.get(DcMotor.class, "BR");

        //These directions essentially define which direction is the 'front' of the robot
        //(and which direction is 'forward')
        this.leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        this.rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        this.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        this.rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        this.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Instantiate and return a new Mecanum drivetrain
        return new MecanumDriveTrain(
                this.leftFrontMotor,
                this.rightFrontMotor,
                this.leftBackMotor,
                this.rightBackMotor,
                gearRatio,
                ticksPerMotorRotation,
                driveBaseLengthInches,
                driveBaseWidthInches,
                wheelRadiusInches
        );
    }
    private Arm initializeArm(){
        this.armMotor = hardwareMap.get(DcMotor.class, "ARM");
        this.armMotor.setDirection(DcMotor.Direction.REVERSE);
        this.starMotor = hardwareMap.get(DcMotor.class, "SM");
        this.starMotor.setDirection(DcMotor.Direction.REVERSE);

        return new SpinnerArm(this.armMotor, this.starMotor);
    }
    private TableSpinner initializeTableSpinner(){
        this.tableSpinnerMotor = hardwareMap.get(DcMotor.class, "spinner");
        this.tableSpinnerMotor.setDirection(DcMotor.Direction.FORWARD);
        this.tableSpinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return new SingleMotorTableSpinner(this.tableSpinnerMotor);
    }

    private VisionSystem initializeVisionSystem(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        return new OpenCvVisionSystem(this.webcam);
    }

    private PositionTracker initializePositionTracker(){
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        return new IMUPositionTracker(this.imu);
    }

    @Override
    public void moveStraightInches(double distanceInches, double power){
        this.telemetry.addData("Command: ","MoveStraight");
        this.telemetry.addData("Distance: ", distanceInches);
        this.telemetry.addData("Power: ", power);
        this.telemetry.update();
        this.driveTrain.moveStraightInches(distanceInches, power);
    }

    @Override
    public void moveSidewaysInches(double distanceInches, double power){
        this.telemetry.addData("Command: ","MoveSideways");
        this.telemetry.addData("Distance: ", distanceInches);
        this.telemetry.addData("Power: ", power);
        this.telemetry.update();
        this.driveTrain.moveSidewaysInches(distanceInches, power);
    }

    @Override
    public void turnAngleDegrees(double angleDegrees, double power){
        this.telemetry.addData("Command: ","TurnAngle");
        this.telemetry.addData("Distance: ", angleDegrees);
        this.telemetry.addData("Power: ", power);
        this.telemetry.update();
        this.driveTrain.turnToAngleDegrees(angleDegrees, power);
    }

    //No Arm yet on the software test bot
    @Override
    public void moveArmPosition(int position){
        this.telemetry.addData("Command: ","MoveArmPosition");
        this.telemetry.addData("Position: ", position);
        this.telemetry.update();
        if(position < 0)
            this.arm.moveToTier(0, 0.0);
        else
            this.arm.moveToTier(position, 1.0);

    }

    @Override
    public void turnArmSpinnerTimed(double timeSeconds, int direction, double power){
        this.telemetry.addData("Command: ","TimedTurnArmSpinner");
        this.telemetry.addData("Time: ", timeSeconds);
        this.telemetry.addData("Power: ", power);
        this.telemetry.update();
        this.arm.cargoSpinTimed(direction, timeSeconds, power);
    }

    @Override
    public void turnArmSpinner(int direction, double power) {
        this.telemetry.addData("Command: ","TimedTurnArmSpinner");
        this.telemetry.addData("Power: ", power);
        this.telemetry.update();
        this.arm.cargoSpin(direction, power);
    }

    @Override
    public void turnSpinnerTimed(double timeSeconds, double power, int direction){
        this.telemetry.addData("Command: ","TurnSpinnerTimed");
        this.telemetry.addData("Time: ", timeSeconds);
        this.telemetry.addData("Power: ", power);
        this.telemetry.addData("Direction: ", direction);
        this.telemetry.update();
        spinner.turnSpinnerTimed(timeSeconds, power, direction);
    }

    @Override
    public void turnSpinner(double power, int direction) {
        this.telemetry.addData("Command: ","TurnSpinner");
        this.telemetry.addData("Power: ", power);
        this.telemetry.addData("Direction: ", direction);
        this.telemetry.update();
        spinner.turnSpinner(power, direction);
    }

    @Override
    public void saveHeading() {
        double currentAngle = this.positionTracker.estimateHeading();
        try {
            FileWriter file = new FileWriter("/sdcard/tmp/SavedHeading.txt");
            BufferedWriter writer = new BufferedWriter(file);
            writer.write(Double.toString(currentAngle));
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    @Override
    public void moveCollectorArm(int position, double power) {
        this.arm2.moveColectorArm(position,power);
    }

    @Override
    public int ScanBarCode(){
        this.telemetry.addData("Command: ", "ScanBarCode");
        this.telemetry.update();
        return this.visionSystem.scanBarCode();
    }

    @Override
    public void stopCameraStream() {
        this.visionSystem.stopVisionStream();
    }
}
