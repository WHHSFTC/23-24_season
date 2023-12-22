package org.firstinspires.ftc.teamcode;
//
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
public class SlidesPID {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double Kp = 0.005;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    private double integral = 0.0;
    private double prevError = 0.0;
    private double state = 0.0;
    private double runTime = 0.0;


    void update(double state, double runTime) {
        this.state = state;
        this.runTime = runTime;
    }

    double calculatePower(double target) {
        double error = target-state;
        double power = 0.0;
        integral += (error-prevError)*runTime;
        power = Kp*(error) + Ki*(integral) + Kd*((error-prevError)/runTime);

        prevError = error;
        return power;
    }
}
