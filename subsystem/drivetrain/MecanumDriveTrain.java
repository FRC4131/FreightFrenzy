package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Vector;

public class MecanumDriveTrain implements DriveTrain
{
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    static int numWheels = 4;
    double driveBaseWidthInches; //The distance between left and right centers side wheels (in inches)
    double driveBaseLengthInches; //The distance between front and back wheel centers (in inches)
    double wheelRadiusInches; //radius of the Mecanum wheels
    double ticksPerWheelRotation; //how many encoder ticks per 1 wheel rotation

    public MecanumDriveTrain(DcMotor inputFrontLeftMotor,
                             DcMotor inputFrontRightMotor,
                             DcMotor inputBackLeftMotor,
                             DcMotor inputBackRightMotor,
                             double inputGearRatio,
                             double inputTicksPerMotorRotation,
                             double inputDriveBaseLengthInches,
                             double inputDriveBaseWidthInches,
                             double inputWheelRadiusInches)
    {
        this.frontLeftMotor = inputFrontLeftMotor;
        this.frontRightMotor = inputFrontRightMotor;
        this.backLeftMotor = inputBackLeftMotor;
        this.backRightMotor = inputBackRightMotor;

        this.ticksPerWheelRotation = inputTicksPerMotorRotation * inputGearRatio;
        this.driveBaseLengthInches = inputDriveBaseLengthInches;
        this.driveBaseWidthInches = inputDriveBaseWidthInches;
        this.wheelRadiusInches = inputWheelRadiusInches;
    }

    //Convert an angular displacement (in radians) to encoder ticks
    private int radiansToTicks(double angularDisplacementRad){
        return (int)(this.ticksPerWheelRotation * angularDisplacementRad / (2.0 * Math.PI));
    }

    @Override
    public void moveStraightInches(double distanceInches, double power) {
        this.moveDisplacement(distanceInches, 0.0, 0.0, power);
    }

    @Override
    public void moveSidewaysInches(double distanceInches, double power) {
        this.moveDisplacement(0.0, distanceInches, 0.0, power);
    }

    @Override
    public void turnToAngleDegrees(double angleDegrees, double power) {
        double angleRadians = angleDegrees * Math.PI/180.0;
        this.moveDisplacement(0.0, 0.0, angleRadians, power);
    }

    @Override
    public void moveVectorDirection(double v_x, double v_y){
        //unused for now
    }

    private void moveDisplacement(double disp_x, double disp_y, double disp_omega, double power){
        Vector<Double> wheelAngularDisplacementsRad = this.forwardKinematic(disp_x, disp_y, disp_omega);
        Vector<Integer> wheelTicks = new Vector<>();

        for (int i = 0; i < numWheels; i++){
            int ticks = this.radiansToTicks(wheelAngularDisplacementsRad.get(i));
            wheelTicks.add(i, ticks);
        }

        this.setMotorTicks(wheelTicks, power);
    }

    //The kinematic model describes the Mecanum wheel angular velocities in terms of the robot velocities, but
    // this could also be used to relate Mecanum wheel angular positions and the robot positions.
    private Vector<Double> forwardKinematic(double v_x, double v_y, double omega_z)
    {
        Vector<Double> wheelOmega = new Vector<>();
        double l_sum = this.driveBaseLengthInches / 2.0 + this.driveBaseWidthInches / 2.0;

        wheelOmega.add(0, 1 / this.wheelRadiusInches *  (v_x - v_y - (l_sum) * omega_z));
        wheelOmega.add(1, 1 / this.wheelRadiusInches *  (v_x + v_y + (l_sum) * omega_z));
        wheelOmega.add(2, 1 / this.wheelRadiusInches *  (v_x + v_y - (l_sum) * omega_z));
        wheelOmega.add(3, 1 / this.wheelRadiusInches *  (v_x - v_y + (l_sum) * omega_z));

        return wheelOmega;
    }

    private void setMotorTicks(Vector<Integer> ticks, double power){
        //Setting the DcMotors to run using the encoders engages the DcMotor's built-in
        // PID controller when setting motor angular speeds (note: the default PID values
        // may be perform poorly if the robot frame is very heavy)
        this.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.frontLeftMotor.setTargetPosition(ticks.get(0));
        this.frontRightMotor.setTargetPosition(ticks.get(1));
        this.backLeftMotor.setTargetPosition(ticks.get(2));
        this.backRightMotor.setTargetPosition(ticks.get(3));

        this.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.frontLeftMotor.setPower(power);
        this.frontRightMotor.setPower(power);
        this.backLeftMotor.setPower(power);
        this.backRightMotor.setPower(power);

        //Wait until the encoder positions have been reached
        while (encodersBusy()){}

        //set the motors to zero to have them stop moving (might need to check motor ZERO condition)
        this.frontLeftMotor.setPower(0);
        this.frontRightMotor.setPower(0);
        this.backLeftMotor.setPower(0);
        this.backRightMotor.setPower(0);
    }

    private boolean encodersBusy(){
        if ((this.frontRightMotor.isBusy())
            & (this.frontLeftMotor.isBusy())
            & (this.backLeftMotor.isBusy())
            & (this.backRightMotor.isBusy()))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
