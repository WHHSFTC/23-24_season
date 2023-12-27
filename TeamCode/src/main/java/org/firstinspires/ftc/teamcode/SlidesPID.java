package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
public class SlidesPID {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double Kp = 0.0025;
    public static double Ki = 0.0;
    public static double Kd = 0.05;
    public static double kGrav = 0.00001;
    public static double kFric = 1.0;
    public static double kIntercept = 0.0;
    private double integral = 0.0;
    private double prevError = 0.0;
    private double prevTarget = 0.0;
    private double state = 0.0;
    private double runTime = 0.0;

    void update(double state, double runTime) {
        this.state = state;
        this.runTime = runTime;
    }

    double calculatePower(double target) {
        double error = target-state;
        double power = 0.0;
        double deltaTarget = target-prevTarget;
        integral += error*runTime;
        power = (error*Kp + integral*Ki + ((error - prevError)/runTime)*Kd) * kFric + kGrav * state + kIntercept;
        prevError = error;
        prevTarget = target;
        return power;
    }
}
