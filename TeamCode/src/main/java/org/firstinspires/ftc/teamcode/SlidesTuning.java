package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp
public class SlidesTuning extends OpMode {
    public static double power = 0.0;
    FtcDashboard dashboard;
    DcMotor ls;
    DcMotor rs;
    VoltageSensor voltageSensor;
    TelemetryPacket packet;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        ls = hardwareMap.get(DcMotor.class, "motorLS");
        rs = hardwareMap.get(DcMotor.class, "motorRS");
        rs.setDirection(DcMotorSimple.Direction.REVERSE);
        voltageSensor=hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {
        ls.setPower(power);
        rs.setPower(power);
        telemetry.addData("Voltage", voltageSensor.getVoltage());
        telemetry.update();
        packet.put("Voltage", voltageSensor.getVoltage());
        packet.put("height", ls.getCurrentPosition());
        packet.put("power", power);
        dashboard.sendTelemetryPacket(packet);
    }
}
