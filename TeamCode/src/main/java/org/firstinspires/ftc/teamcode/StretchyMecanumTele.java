package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.DelaysAndAutoms.updateDelays;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCameraFactory;

@TeleOp
public class StretchyMecanumTele extends OpMode {

    public static double strafeScalar = 1.1;
    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;

    public void init() {

        //DC Motors
        rf = hardwareMap.get(DcMotor.class, "motorRF");
        lf = hardwareMap.get(DcMotor.class, "motorLF");
        rb = hardwareMap.get(DcMotor.class, "motorRB");
        lb = hardwareMap.get(DcMotor.class, "motorLB");


        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void loop() {
        rf.getCurrentPosition();
        lf.getCurrentPosition();
        rb.getCurrentPosition();
        lb.getCurrentPosition();

        childLoop();
    }
    public void childLoop(){

        double ly = gamepad1.left_stick_y;
        double lx = -gamepad1.left_stick_x * strafeScalar;
        double ry = gamepad2.right_stick_y;
        double rx = -gamepad2.right_stick_x * strafeScalar;

        double preRF = ry - rx;
        double preRB = ry + rx;
        double preLF = -ly - lx;
        double preLB = -ly + lx;

        double max = Math.max(Math.max(Math.max(Math.max(preRF,preRB), preLB), preLF), 1);

        double postRF = preRF/max;
        double postRB = preRB/max;
        double postLF = preLF/max;
        double postLB = preLB/max;

        rf.setPower(postRF);
        rb.setPower(postRB);
        lf.setPower(postLF);
        lb.setPower(postLB);

        telemetry.addData("rf", postRF);
        telemetry.addData("rb", postRB);
        telemetry.addData("lf", postLF);
        telemetry.addData("lb", postLB);
        telemetry.addData("rx", rx);
        telemetry.addData("ry", ry);
        telemetry.addData("lx", lx);
        telemetry.addData("ly", ly);
        telemetry.update();
    }

    public void stop(){
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }
}
