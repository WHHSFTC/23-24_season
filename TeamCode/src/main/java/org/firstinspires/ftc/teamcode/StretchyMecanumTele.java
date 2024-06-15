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
    public static double rScalar = 1.3;
    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;

    IMU imu;

    int methodIdx = 0;
    boolean prevLeft = false;
    boolean prevRight = false;

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

        imu = hardwareMap.get(IMU.class, "imu");
    }
    public void loop() {
        rf.getCurrentPosition();
        lf.getCurrentPosition();
        rb.getCurrentPosition();
        lb.getCurrentPosition();

        childLoop();
    }
    public void childLoop(){

        double ly = 0, lx = 0, ry = 0, rx = 0;

        //increment or decrement method index as needed
        if(gamepad1.right_bumper && !prevRight) {
            prevRight = true;
            methodIdx++;
        } else if(!gamepad1.right_bumper){
            prevRight = false;
        }
        if(gamepad1.left_bumper && !prevLeft) {
            prevLeft = true;
            methodIdx--;
        } else if(!gamepad1.left_bumper){
            prevLeft = false;
        }

        switch(methodIdx % 4) {
            case 0:
                telemetry.addData("Method:", "Double Joystick");
                ly = gamepad1.left_stick_y;
                lx = -gamepad1.left_stick_x * strafeScalar;
                ry = gamepad1.right_stick_y;
                rx = -gamepad1.right_stick_x * strafeScalar;
                break;
            case 1:
                telemetry.addData("Method:", "Enhanced Mecanum");
                double y = gamepad1.left_stick_y;
                double x = -gamepad1.left_stick_x * strafeScalar;
                double r = -gamepad1.right_stick_x * rScalar;
                double ext = gamepad1.right_stick_y;
                ly = y + r;
                lx = x - ext;
                ry = y - r;
                rx = x + ext;
                break;
            case 2:
                telemetry.addData("Method:", "Buttons");
                x = 0; // positive x is right
                y = 0; // positive y is forward
                ly = y;
                lx = -x * strafeScalar;
                ry = y;
                rx = -x * strafeScalar;
                break;
            case 3:
                telemetry.addData("Method:", "Two Controllers");
                ly = gamepad1.left_stick_y;
                lx = -gamepad1.left_stick_x * strafeScalar;
                ry = gamepad2.right_stick_y;
                rx = -gamepad2.right_stick_x * strafeScalar;
                break;
        }

        //reset imu
        if (gamepad1.back) {
            imu.resetYaw();
        }

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

        telemetry.addData("rx", rx);
        telemetry.addData("ry", ry);
        telemetry.addData("lx", lx);
        telemetry.addData("ly", ly);
        telemetry.addData("roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS));
        telemetry.update();
    }

    public void stop(){
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }
}
