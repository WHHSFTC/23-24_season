package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.DelaysAndAutoms.updateDelays;

import java.util.List;


@Disabled
@TeleOp
@Config
abstract public class CenterStageOpMode extends OpMode {
    boolean blue = true, isRightSideHardForCameraToSee = true;

    static SampleMecanumDrive drive;
    FtcDashboard dashboard;
    static TelemetryPacket packet;
    List<LynxModule> bothHubs;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime totalRunTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double timePerLoop;
    double loopCumulativeTime = 0.0;
    double loopCounter = 0.0;

    IMU imu;
    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;
    DcMotor intake;
    DcMotor conveyor;
    DcMotor ls;
    DcMotor rs;

    public static double slidePositionTarget = 0.0;
    double slideSavedPosition = 1100.0;

    public static double intakeUpPos = 0.0;
    public static double intakeDownPos = 0.71;
    public static double intakeStackPos = 0.54;
    public static double intakeSecondStackPos = 0.57;
    public static double intakeLeftUpScale = 0.0;
    public static double intakeLeftDownScale = 1.0;
    public static double intakeRightDownScale = 1.0;
    public static double intakeRightUpScale = 0.0;

    public static double armOutPos = 0.0;
    public static double armInPos = 0.94;
    public static double plungerGrabPos = 0.0;
    public static double plungerReleasePos = 1.0;
    public static double dronePos1 = 0.35;
    public static double dronePos2 = 0.95;
    public static double distBackdrop = 4.35; //inches
    public static double leftDSOffset = 1.25;
    //Servo armRight;
    public Servo armLeft;
    public Servo pRight;
    public Servo pLeft;
    public Servo intakeRight;
    public Servo intakeLeft;
    DelaysAndAutoms slideUpdate;
    DelaysAndAutoms slideDelay;
    Servo droneLauncher;

    TouchSensor slidesLimit;
    DistanceSensor rightDS;
    DistanceSensor leftDS;
    DelaysAndAutoms waitFor;

    VoltageSensor voltageSensor;
    OpenCvWebcam webcam;
    SlidesPID slidesPidRight;
    SlidesPID slidesPidLeft;
    Gamepad gamepad1prev = new Gamepad();
    Gamepad gamepad2prev = new Gamepad();

    @Override
    public void init() {
        bothHubs = hardwareMap.getAll(LynxModule.class);
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        slidesPidRight = new SlidesPID();
        slidesPidLeft = new SlidesPID();

        imu = hardwareMap.get(IMU.class, "imu");

        //DC Motors
        rf = hardwareMap.get(DcMotor.class, "motorRF");
        lf = hardwareMap.get(DcMotor.class, "motorLF");
        rb = hardwareMap.get(DcMotor.class, "motorRB");
        lb = hardwareMap.get(DcMotor.class, "motorLB");
        intake = hardwareMap.get(DcMotor.class, "motorIntake");
        conveyor = hardwareMap.get(DcMotor.class, "motorConveyor");
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
        voltageSensor=hardwareMap.voltageSensor.iterator().next();

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
        intakeRight.setDirection(Servo.Direction.REVERSE);

        // intakeLeft.setPosition(1.0);
        // intakeRight.setPosition(1.0);

        armLeft.scaleRange(0.01, 0.245);
        //armRight.scaleRange(0.0, 0.245);
//
        armLeft.setPosition(0.0);

        pRight.scaleRange(0.65, 0.77);
        pLeft.scaleRange(0.51,0.67);

        pRight.setPosition(1.0);
        pLeft.setPosition(1.0);

        droneLauncher.scaleRange(0.3, 0.9);
        droneLauncher.setPosition(dronePos2);

        //camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "outputCamera"), cameraMonitorViewId);
        webcam.setMillisecondsPermissionTimeout(5000);
    }

    @Override
    public void start() {
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    final public void loop() {
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

        intakeRight.scaleRange(intakeRightUpScale, intakeRightDownScale);
        intakeLeft.scaleRange(intakeLeftUpScale, intakeLeftDownScale);

        for (LynxModule hub : bothHubs) {
            hub.clearBulkCache();
        }
        //bulk read
        rf.getCurrentPosition();
        lf.getCurrentPosition();
        rb.getCurrentPosition();
        lb.getCurrentPosition();
        ls.getCurrentPosition();
        rs.getCurrentPosition();
        slidesLimit.isPressed();
        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        childLoop();
        updateDelays();
        if(slideUpdate != null) {
            slidePositionTarget = slideUpdate.updateVariable();
            if(slideUpdate.delayTimer.milliseconds() >= slideUpdate.delay){
                slideUpdate = null;
            }
        }

        if(slideDelay != null) {
            slidePositionTarget = slideDelay.updateVariable();
            if(slideDelay.delayTimer.milliseconds() >= slideDelay.delay){
                slideDelay = null;
            }
        }

        telemetry.addData("rs position", rs.getCurrentPosition());
        telemetry.addData("ls position", ls.getCurrentPosition());
        telemetry.addData("time per loop", timePerLoop);
        telemetry.update();
    }

    @Override
    public void stop() {
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        intake.setPower(0);
        ls.setPower(0);
        rs.setPower(0);
    }

    public void childLoop() {

    }
}
