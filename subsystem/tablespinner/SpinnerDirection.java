package org.firstinspires.ftc.teamcode.subsystem.tablespinner;

import android.widget.Spinner;

public enum SpinnerDirection {
    FORWARD(1), REVERSE(-1);

    private int dir;

    SpinnerDirection(int inputDir){
        this.dir = inputDir;
    }
    public int getVal(){
        return this.dir;
    }
}
