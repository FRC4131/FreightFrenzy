package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.visionsystem.BareBonesVisionSystem;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.positiontracker.IMUPositionTracker;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.subsystem.tablespinner.SingleMotorTableSpinner;
import org.firstinspires.ftc.teamcode.subsystem.tablespinner.TableSpinner;
import org.firstinspires.ftc.teamcode.subsystem.visionsystem.VisionSystem;
import org.openftc.easyopencv.OpenCvWebcam;

/* Software test robot implementation.
    Should have a basic Mecanum drivetrain, a spinner and webcam,
    and should satisfy our Robot interface */

public class SoftwareBot implements Robot{
    //High level robot subsystems
    DriveTrain driveTrain;
    TableSpinner spinner;
    VisionSystem visionSystem;
    PositionTracker positionTracker;

    //Robot messaging/debugging and hardware mapping components
    Telemetry telemetry;
    HardwareMap hardwareMap;

    /*LOW LEVEL ROBOT COMPONENTS (i.e. parts that makeup the high-level components (ex: motors, servos, webcam, etc.)*/
    //Robot driveTrain components and parameters
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    static double gearRatio = 20;  //i.e. a gearbox with 20:1 gear ratio
    static double ticksPerMotorRotation = 24;  //i.e. (6 encoder cycles)/(motor revolution) * (4 encoder ticks or events)/(encoder cycle) = (24 ticks)/(motor revolution) -- see Torquenado encoder Specs
    static double driveBaseLengthInches = 10;
    static double driveBaseWidthInches = 11.5;
    static double wheelRadiusInches = 1.929135; // "Tetrix MAX" wheel radius: 49 mm -> approx 1.93 inches

    //Table spinner components
    DcMotor tableSpinnerMotor = null;

    //Vision system components (need to set to null?)
    OpenCvWebcam webcam;

    //Position tracker components (for now we only care about heading)
    BNO055IMU imu;

    public SoftwareBot(Telemetry inputTelemetry, HardwareMap inputHardwareMap){
        this.telemetry = inputTelemetry;
        this.hardwareMap = inputHardwareMap;
        this.positionTracker = initializePositionTracker();
        this.driveTrain = initializeDriveTrain();
        this.spinner = initializeTableSpinner();
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

    private TableSpinner initializeTableSpinner(){
        this.tableSpinnerMotor = hardwareMap.get(DcMotor.class, "spinner");
        this.tableSpinnerMotor.setDirection(DcMotor.Direction.FORWARD);
        this.tableSpinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return new SingleMotorTableSpinner(this.tableSpinnerMotor);
    }

    private VisionSystem initializeVisionSystem(){
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        return new OpenCvVisionSystem(this.webcam);
        return new BareBonesVisionSystem();
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
    }

    @Override
    public void turnArmSpinnerTimed(double timeSeconds, int direction, double power){
        this.telemetry.addData("Command: ","TimedTurnArmSpinner");
        this.telemetry.addData("Time: ", timeSeconds);
        this.telemetry.addData("Power: ", power);
        this.telemetry.update();
    }

    @Override
    public void turnSpinnerTimed(double timeSeconds, double power, int direction){
        this.telemetry.addData("Command: ","TimedTurnSpinner");
        this.telemetry.addData("Time: ", timeSeconds);
        this.telemetry.addData("Power: ", power);
        this.telemetry.addData("Direction: ", direction);
        this.telemetry.update();
        spinner.turnSpinnerTimed(timeSeconds, power, direction);
    }

    @Override
    public int ScanBarCode(){
        this.telemetry.addData("Command: ", "ScanBarCode");
        this.telemetry.update();
        return this.visionSystem.scanBarCode();
    }
}
