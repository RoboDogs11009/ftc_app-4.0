package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;

import static java.lang.Math.abs;

public class potentiometer {
    public double degree(double voltage){
        double degree=81*abs(3.333-voltage);

        return degree;
    }
}
