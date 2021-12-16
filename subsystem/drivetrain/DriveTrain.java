package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

public interface DriveTrain {
    void moveStraightInches(double distanceInches, double power);
    void moveSidewaysInches(double distanceInches, double power);
    void turnToAngleDegrees(double angleDegrees, double power);
    void moveVectorDirection(double v_x, double v_y);
}
