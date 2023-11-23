package org.firstinspires.ftc.teamcode;
//
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
public class SlidesPID {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double Kp = 0.005;
    public static double Ki = 0;
    public static double Kd = 0.0;
    public static double calculatePower(double target, double state, double runTime) {
        double prevError = 0;
        double error = target-state;
        double power = 0.0;
        power = Kp*(error) + Ki*((error-prevError)*runTime) + Kd*((error-prevError)/runTime);

        prevError = error;
        return power;
    }
}
