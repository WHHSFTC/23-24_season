package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Disabled
@TeleOp
abstract public class CenterStageOpMode extends OpMode {

    FtcDashboard dashboard;
    static TelemetryPacket packet;
    List<LynxModule> bothHubs;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double timePerLoop;
    public double loopCumulativeTime = 0.0;
    public double loopCounter = 0.0;

    DcMotor rf;
    public double rfPosition;
    DcMotor lf;
    public double lfPosition;
    DcMotor rb;
    public double rbPosition;
    DcMotor lb;
    public double lbPosition;
    DcMotor intake;
    DcMotor ls;
    public double lsPosition;
    DcMotor rs;
    public double rsPosition;

    //Servo armRight;
    Servo armLeft;
    Servo pRight;
    Servo pLeft;
    Servo intakeRight;
    Servo intakeLeft;
    Servo droneLauncher;

    TouchSensor slidesLimit;
    DistanceSensor rightDS;
    DistanceSensor leftDS;

    @Override
    public void init() {
        bothHubs = hardwareMap.getAll(LynxModule.class);
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        //DC Motors
        rf = hardwareMap.get(DcMotor.class, "motorRF");
        lf = hardwareMap.get(DcMotor.class, "motorLF");
        rb = hardwareMap.get(DcMotor.class, "motorRB");
        lb = hardwareMap.get(DcMotor.class, "motorLB");
        intake = hardwareMap.get(DcMotor.class, "motorIntake");
        ls = hardwareMap.get(DcMotor.class, "motorLS");
        rs = hardwareMap.get(DcMotor.class, "motorRS");

        //Servos
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        //armRight = hardwareMap.get(Servo.class, "armRight");
        pRight = hardwareMap.get(Servo.class, "plungerRight");
        pLeft = hardwareMap.get(Servo.class, "plungerLeft");
        droneLauncher = hardwareMap.get(Servo.class, "drone");

        //Sensors
        slidesLimit = hardwareMap.get(TouchSensor.class, "slidesLimit");
        rightDS = hardwareMap.get(DistanceSensor.class, "rightDS");
        leftDS = hardwareMap.get(DistanceSensor.class, "leftDS");

        for(LynxModule hub : bothHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);
        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");

        intakeLeft.scaleRange(0.0, 0.65);
        intakeRight.scaleRange(0.0, 1.0);

        intakeRight.setDirection(Servo.Direction.REVERSE);

        // intakeLeft.setPosition(1.0);
        // intakeRight.setPosition(1.0);

        armLeft.scaleRange(0.0, 0.245);
        //armRight.scaleRange(0.0, 0.245);

        armLeft.setPosition(0.0);

        pRight.scaleRange(0.68, 0.77);
        pLeft.scaleRange(0.57,0.67);

        pRight.setPosition(1.0);
        pLeft.setPosition(1.0);

        droneLauncher.scaleRange(0.3, 0.9);
        droneLauncher.setPosition(0);
    }

    @Override
    public void loop() {

        //timing stuff
        timePerLoop = timer.milliseconds();
        timer.reset();
        loopCounter++;
        loopCumulativeTime += timePerLoop;

        if (loopCumulativeTime >= 1000) {
            telemetry.addData("Time per Loop", "Time per Loop: " + loopCumulativeTime/loopCounter);
            loopCounter = 0.0;
            loopCumulativeTime = 0.0;
        }


        for (LynxModule hub : bothHubs) {
            hub.clearBulkCache();
        }

        rfPosition = rf.getCurrentPosition();
        lfPosition = lf.getCurrentPosition();
        rbPosition = rb.getCurrentPosition();
        lbPosition = lb.getCurrentPosition();
        lsPosition = ls.getCurrentPosition();
        rsPosition = rs.getCurrentPosition();

    }

    @Override
    public void stop() {
        super.stop();
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        intake.setPower(0);
        ls.setPower(0);
        rs.setPower(0);
    }
}
