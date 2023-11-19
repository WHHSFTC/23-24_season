package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
public class SlidesPID {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double Kp = 0.015;
    public static double Ki = 0;
    public static double Kd = 0.0;
    public static double calculatePower(double target, double state, double time) {
        double power = 0;
        power = Kp*(target-state) + Ki*((target-state)*time) + Kd*((target-state)/time);

        return power;
    }
}
