package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class DSpid {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double dkp = 0.2;
    public static double dkd = 0.0;
    public static double dki = 0.0;
    public static double dMaxPower = 2.0;
    public static double rkp = 1.5;
    public static double rkd = 0.0;
    public static double rki = 0.0;
    public static double rkf = 0.1;
    public static double zero = 0.002;
    private static double prevError = 0.0;
    public static double integral = 0.0;

    public static double distancePID(double dr, double dl, double elapsedTime, double target){
        double averageDistance = Math.min(dr, dl);
        double power = 0.0;
        double error = averageDistance - target;
        integral += error*elapsedTime;
        power = error*dkp + integral*dki + ((error - prevError)/elapsedTime)*dkd;
        prevError = error;
        return Math.min(dMaxPower, power);
    }

    public static double anglePID(double state, double elapsedTime, double target){
        double power = 0.0;
        double error = state - target;
        double ff = 0.0;
        if (error > zero) {ff = 0.1;} else if (error < -zero) {ff = -0.1;}
        integral += error*elapsedTime;
        power = error*rkp + integral*rki + ((error - prevError)/elapsedTime)*rkd + ff;
        prevError = error;
        return power;
    }
}
