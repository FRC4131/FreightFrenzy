package org.firstinspires.ftc.teamcode.robot;

public interface Robot
{
    /*We specify ALL our potential robot behaviors here that the high level scripts
        use as an interface to  accomplish tasks (They are roughly separated
        by high level component.) Whether or the robot actually has the hardware or capability to
        execute the behaviors shouldn't matter as long as we satisfy this interface. */
    //Drivetrain/Movement behaviors
    void moveStraightInches(double distanceInches, double power);
    void moveSidewaysInches(double distanceInches, double power);
    void turnAngleDegrees(double angleDegrees, double power);

    //Spinner-Arm specific behaviors
    void moveArmPosition(int position);
    void turnArmSpinnerTimed(double timeSeconds, int direction, double power);

    //Turntable spinner behaviors
    void turnSpinnerTimed(double timeSeconds, double power, int direction);

    //Vision system behaviors
    int ScanBarCode();
}
