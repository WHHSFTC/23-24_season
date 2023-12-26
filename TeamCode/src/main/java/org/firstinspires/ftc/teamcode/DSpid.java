package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class DSpid {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double dkp = 0.2;
    public static double dkd = 0.0;
    public static double dki = 0.0;
    public static double rkp = 0.9;
    public static double rkd = 0.0;
    public static double rki = 0.0;
    private static double prevError = 0.0;
    public static double integral = 0.0;

    public static double distancePID(double dr, double dl, double elapsedTime, double target){
        double averageDistance = Math.min(dr, dl);
        double power = 0.0;
        double error = averageDistance - target;
        integral += error*elapsedTime;
        power = error*dkp + integral*dki + ((error - prevError)/elapsedTime)*dkd;
        prevError = error;
        return power;
    }

    public static double anglePID(double state, double elapsedTime, double target){
        double power = 0.0;
        double error = state - target;
        integral += error*elapsedTime;
        power = error*rkp + integral*rki + ((error - prevError)/elapsedTime)*rkd;
        prevError = error;
        return power;
    }
}
