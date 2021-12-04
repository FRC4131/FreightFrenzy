package org.firstinspires.ftc.teamcode.subsystem.arm;

public interface Arm {
    void moveToTier(int tier, double power);
    void cargoSpin(int direction, double time, double power);
}
