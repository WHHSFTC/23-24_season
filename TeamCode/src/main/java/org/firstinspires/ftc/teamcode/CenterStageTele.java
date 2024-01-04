package org.firstinspires.ftc.teamcode;//

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
@TeleOp
public class CenterStageTele extends OpMode{

    FtcDashboard dashboard;
    TelemetryPacket packet;
    //public static MultipleTelemetry dashTelemetry = new MultipleTelemetry();
    Gamepad gamepad1prev = new Gamepad();
    Gamepad gamepad2prev = new Gamepad();

    IMU imu;
    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;
    DcMotor intake;
    DcMotor ls;
    DcMotor rs;

    public static double slidePositionTarget = 0.0;
    public static double slidesff = 0.0;
    public static double slideTargetGain = 300.0;
    public static double slideMin = 0.0;
    public static double slideMax = 2200.0;
    double distancePower;
    double anglePower;
    boolean slidesPressed = true;
    boolean dpadDownPressed;
    double slideSavedPosition = 1100.0;

    public static double intakeUpPos = 0.64;
    public static double intakeDownPos = 0.07;
    public static double intakeStackPos = 0.18;

    public static double armOutPos = 0.01;
    public static double armInPos = 0.94;
    public static double plungerGrabPos = 0.0;
    public static double plungerReleasePos = 1.0;
    public static double dronePos1 = 0.35;
    public static double dronePos2 = 0.95;
    public static double distBackdrop = 6.20;
    double targetYaw = 0.0;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    SlidesPID slidesPidRight;
    SlidesPID slidesPidLeft;
    double timeGap;
    boolean intakeOnGround;
    boolean plungerLClosed;
    boolean plungerRClosed;
    boolean zeroing = false;

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
    public void init(){

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        slidesPidRight = new SlidesPID();
        slidesPidLeft = new SlidesPID();

        imu = hardwareMap.get(IMU.class, "imu");
        rf = hardwareMap.get(DcMotor.class, "motorRF");
        lf = hardwareMap.get(DcMotor.class, "motorLF");
        rb = hardwareMap.get(DcMotor.class, "motorRB");
        lb = hardwareMap.get(DcMotor.class, "motorLB");
        intake = hardwareMap.get(DcMotor.class, "motorIntake");
        ls = hardwareMap.get(DcMotor.class, "motorLS");
        rs = hardwareMap.get(DcMotor.class, "motorRS");

        armLeft = hardwareMap.get(Servo.class, "armLeft");
        //armRight = hardwareMap.get(Servo.class, "armRight");
        pRight = hardwareMap.get(Servo.class, "plungerRight");
        pLeft = hardwareMap.get(Servo.class, "plungerLeft");
        droneLauncher = hardwareMap.get(Servo.class, "drone");

        slidesLimit = hardwareMap.get(TouchSensor.class, "slidesLimit");
        leftDS = hardwareMap.get(DistanceSensor.class, "leftDS");
        rightDS = hardwareMap.get(DistanceSensor.class, "rightDS");

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

        intakeLeft.scaleRange(0.0, 1.0);
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

        //droneLauncher.scaleRange(dronePos1, dronePos2);
        //droneLauncher.setPosition(1.0);
        intakeOnGround = false;
        plungerLClosed = true;
        plungerRClosed = true;

        /*rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE); */

        imu.resetYaw();

        gamepad1prev.copy(gamepad1);
        gamepad2prev.copy(gamepad2);
    }//

    @Override
    public void loop(){
        slidesPidRight.update(rs.getCurrentPosition(), timeGap);
        slidesPidLeft.update(ls.getCurrentPosition(), timeGap);
        //find timeGap
        timeGap = timer.milliseconds();
        timer.reset();

        double scalar = 1.0;

        double y = -gamepad1.left_stick_x; //verticals
        double x = -gamepad1.left_stick_y*1.1; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation

        if(gamepad1.a){
            distancePower = DSpid.distancePID(rightDS.getDistance(DistanceUnit.INCH), leftDS.getDistance(DistanceUnit.INCH), timeGap, distBackdrop);
            anglePower = DSpid.anglePID(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), timeGap, Math.toRadians(0));
            x = x>0.55 ? x : -distancePower;
            r = -anglePower;
            scalar = 0.25;
        }

        //heading lock
        if(gamepad1.b){
            anglePower = DSpid.anglePID(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), timeGap, Math.toRadians(180));
            r = -anglePower;
        }

        telemetry.addData("Pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));
        telemetry.addData("Roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS));
        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        //slides
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (slidesLimit.isPressed()) {
            if (!slidesPressed) {
                ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            slidesPressed = true;
        } else {
            slidesPressed = false;
        }

        if (Math.abs(gamepad2.left_stick_y) > 0.01) {

            if (Math.abs(gamepad2.left_stick_y) > 0.01) {slidePositionTarget -= slideTargetGain * gamepad2.left_stick_y;}

            if (slidePositionTarget < slideMin) {
                slidePositionTarget = slideMin;
            }
            if (slidePositionTarget > slideMax) {
                slidePositionTarget = slideMax;
            }
            if (slidePositionTarget > 600.0) {
                scalar = 1.0 - (Math.sqrt(slidePositionTarget/slideMax)/1.25);
            }
        }

        telemetry.addData("Slide target: ", slidePositionTarget);
        telemetry.addData("Error RS", "Error RS: " + (slidePositionTarget - rs.getCurrentPosition()));
        telemetry.addData("Error LS", "Error LS: " + (slidePositionTarget - ls.getCurrentPosition()));

        rs.setPower(slidesPidRight.calculatePower(slidePositionTarget));
        ls.setPower(slidesPidLeft.calculatePower(slidePositionTarget));

        if (gamepad1.left_bumper && scalar > 0.3){
            scalar = 0.3;
        }

        double preRF = 1.3*r*Math.sqrt(scalar) + y*Math.cbrt(scalar) + x*scalar;
        double preLF = 1.3*r*Math.sqrt(scalar) + y*Math.cbrt(scalar) - x*scalar;
        double preRB = 1.3*r*Math.sqrt(scalar) - y*Math.cbrt(scalar) + x*scalar;
        double preLB = 1.3*r*Math.sqrt(scalar) - y*Math.cbrt(scalar) - x*scalar;


        double max = Math.max(Math.max(Math.max(Math.max(preRF,preRB), preLB), preLF), 1);


            rf.setPower(preRF/max);
            lf.setPower(preLF/max);
            rb.setPower(preRB/max);
            lb.setPower(preLB/max);

        double postRF = preRF/max;
        double postLF = preLF/max;
        double postRB = preRB/max;
        double postLB = preLB/max;

        //arm swings out
        if (gamepad2.x) {
            //armRight.setPosition(0.0);
            armLeft.setPosition(armOutPos);
        }

        //arm swings in
        if (gamepad2.b) {
            //armRight.setPosition(1.0);
            armLeft.setPosition(armInPos);
        }

        //plunger open
        if (gamepad2.y) {
            pRight.setPosition(plungerGrabPos);
            pLeft.setPosition(plungerGrabPos);
            plungerLClosed = false;
            plungerRClosed = false;
        }

        //plunger close
        if (gamepad2.a) {
            pRight.setPosition(plungerReleasePos);
            pLeft.setPosition(plungerReleasePos);
            plungerLClosed = true;
            plungerRClosed = true;
        }

        if (gamepad2.dpad_right && !zeroing) {
            slidePositionTarget = slideSavedPosition;
        }

        if (gamepad2.dpad_down && !zeroing) {
            if (!dpadDownPressed) {
                slideSavedPosition = slidePositionTarget;
                slidePositionTarget = slideMin;
            }
            dpadDownPressed = true;
        } else {
            dpadDownPressed = false;
        }

        //intake spinning
        if (gamepad1.left_trigger > 0.2 && intakeOnGround) {
            intake.setPower(-0.3 * gamepad1.left_trigger); //reverse
        } else if (gamepad1.right_trigger > 0.2 && intakeOnGround) {
            intake.setPower(gamepad1.right_trigger*0.8); //forward
            if (slidePositionTarget < 130.0) {
                slidePositionTarget = 130.0;
            }
        } else {
            intake.setPower(0.0);
        }

        //intake swinging out and swinging in
        if(gamepad1.right_bumper && !gamepad1prev.right_bumper){
                if(intakeOnGround){
                    intakeRight.setPosition(intakeUpPos);
                    intakeLeft.setPosition(intakeUpPos);
                    intakeOnGround = false;
                }else{
                    intakeRight.setPosition(intakeDownPos);
                    intakeLeft.setPosition(intakeDownPos);
                    intakeOnGround = true;
                }
        }

        //stack position intake
        if (gamepad1.dpad_right) {
            if(!intakeOnGround){
                intakeOnGround = true;
            }
            intakeRight.setPosition(intakeStackPos);
            intakeLeft.setPosition(intakeStackPos);
        }

        //reset imu
        if(gamepad1.back){
            imu.resetYaw();
        }

        //manual reset for slides
        if (gamepad2.start) {
            zeroing = true;
        }
        if (zeroing) {
            slidePositionTarget = 0.0;
            rs.setPower(-0.3);
            ls.setPower(-0.3);
        }
        if (slidesLimit.isPressed()) {
            zeroing = false;
        }

        //hammer
        if(gamepad1.y){
            intakeLeft.setPosition(0.8);
            intakeRight.setPosition(0.8);
            DelaysAndAutoms.delayServo(200,intakeLeft, 0.75, intakeDownPos);
            DelaysAndAutoms.delayServo(200,intakeRight, 0.75, intakeDownPos);
        }

        // drone launcher
        if (gamepad2.back) {
            droneLauncher.setPosition(dronePos1);
        } else {
            droneLauncher.setPosition(dronePos2);
        }

        //manual release for plungers
        if(gamepad2.right_bumper){
            if(plungerLClosed){
                pLeft.setPosition(plungerGrabPos);
                plungerLClosed = false;
            }
            else{
                pLeft.setPosition(plungerReleasePos);
                plungerLClosed = true;
            }
        }

        if(gamepad2.left_bumper){
            if(plungerRClosed){
                pRight.setPosition(plungerGrabPos);
                plungerRClosed = false;
            }
            else{
                pRight.setPosition(plungerReleasePos);
                plungerRClosed = true;
            }
        }

        //output automation
        if(gamepad2.dpad_left){
            slidePositionTarget = 200.0;
            rs.setPower(slidesPidRight.calculatePower(slidePositionTarget));
            ls.setPower(slidesPidLeft.calculatePower(slidePositionTarget));
            DelaysAndAutoms.delayServo(100.0, armLeft, armOutPos, armInPos);
            slidePositionTarget = -5.0;
            DelaysAndAutoms.delayMotor(200.0,ls, slidesPidLeft.calculatePower(200.0),
                    slidesPidLeft.calculatePower(slidePositionTarget));
            DelaysAndAutoms.delayMotor(200.0,rs, slidesPidRight.calculatePower(200.0),
                    slidesPidRight.calculatePower(slidePositionTarget));
            DelaysAndAutoms.delayServo(400.0,pRight,plungerReleasePos, plungerGrabPos);
            DelaysAndAutoms.delayServo(400.0,pLeft,plungerReleasePos, plungerGrabPos);
        }

        if(gamepad2.left_stick_y < -0.1 && (slidePositionTarget >= 1000) && (armLeft.getPosition() > 0.8) && !zeroing){
            armLeft.setPosition(armOutPos);
        }

        gamepad1prev.copy(gamepad1);
        gamepad2prev.copy(gamepad2);

        telemetry.addData("rf", postRF);
        telemetry.addData("lf", postLF);
        telemetry.addData("rb", postRB);
        telemetry.addData("lb", postLB);

        telemetry.addData("intake", intake.getPower());
        //telemetry.addData("armRight", "Position: " + armRight.getPosition());
        telemetry.addData("armLeft", "Position: " + armLeft.getPosition());
        telemetry.addData("pRight", "Position: " + pRight.getPosition());
        telemetry.addData("pLeft", "Position: " + pLeft.getPosition());
        telemetry.addData("intakeRight", "Position: " + intakeRight.getPosition());
        telemetry.addData("intakeLeft", "Position: " + intakeLeft.getPosition());
        telemetry.addData("rs", "Power: " + rs.getPower());
        telemetry.addData("ls", "Power: " + ls.getPower());
        telemetry.addData("drone", "Position: " + droneLauncher.getPosition());
        //telemetry.addData("slides target", "target: " + slidePositionTarget);
        telemetry.addData("slides position rs: ", "current rs pos: " + rs.getCurrentPosition());
        telemetry.addData("slides position ls: ", "current ls pos: " + ls.getCurrentPosition());
        telemetry.addData("limit switch", slidesLimit.isPressed());
        telemetry.addData("leftDS", "Distance B from backdrop: " + leftDS.getDistance(DistanceUnit.INCH));
        telemetry.addData("rightDS", "Distance A from backdrop: " + rightDS.getDistance(DistanceUnit.INCH));
        telemetry.update();

        packet.put("slides target", slidePositionTarget);
        packet.put("slides position rs", rs.getCurrentPosition());
        packet.put("slides position ls", ls.getCurrentPosition());
        packet.put("slides power rs", rs.getPower());
        packet.put("slides power ls", ls.getPower());
        packet.put("slides limit pressed", slidesLimit.isPressed());
        packet.put("slide saved position", slideSavedPosition);
        packet.put("scalar", scalar);
        packet.put("target distanceBackdrop", distBackdrop);
        packet.put("leftDistBackdrop", leftDS.getDistance(DistanceUnit.INCH));
        packet.put("rightDistBackdrop", rightDS.getDistance(DistanceUnit.INCH));
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop(){
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
