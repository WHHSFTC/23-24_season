package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;

import java.util.ArrayList;

@Config
@TeleOp
public class CenterStageTele extends OpMode{
//
    FtcDashboard dashboard;
    TelemetryPacket packet;
    //public static MultipleTelemetry dashTelemetry = new MultipleTelemetry();

    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;
    DcMotor intake;
    DcMotor ls;
    DcMotor rs;

    public static double slidePositionTarget = 0.0;
    public static double slidesff = 0.0;
    public static double slideTargetGain = 100.0;
    public static double slideMin = 0.0;
    public static double slideMax = 1800.0;
    boolean dlidesPressed;
    boolean dpadDownPressed;
    double slideSavedPosition = 900.0;

    public static double intakeUpPos = 1.0;
    public static double intakeDownPos = 0.12;
    public static double intakeStackPos = 0.25;

    public static double armOutPos = 0.1;
    public static double armInPos = 1.0;
    public static double plungerGrabPos = 0.0;
    public static double plungerReleasePos = 1.0;
    public static double dronePos1 = 0.2;
    public static double dronePos2 = 0.9;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double timegap;


    //Servo armRight;
    Servo armLeft;
    Servo pRight;
    Servo pLeft;
    Servo intakeRight;
    Servo intakeLeft;
    Servo droneLauncher;

    TouchSensor slidesLimit;

    @Override
    public void init(){

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

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

        droneLauncher.scaleRange(dronePos1, dronePos2);
        droneLauncher.setPosition(0);

        /*rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE); */
    }

    @Override
    public void loop(){
        //find timegap
        timegap = timer.milliseconds();
        timer.reset();

        double scalar = 1.0;

        double y = -gamepad1.left_stick_x; //verticals
        double x = -gamepad1.left_stick_y*1.3; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation


        //slides
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (slidesLimit.isPressed()) {
            if (!dlidesPressed) {
                ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            dlidesPressed = true;
        } else {
            dlidesPressed = false;
        }

        //if (Math.abs(gamepad2.left_stick_y) > 0.01) {

            if (gamepad1.dpad_up) {slidePositionTarget -= slideTargetGain * gamepad1.right_stick_y;}

            if (slidePositionTarget < slideMin) {
                slidePositionTarget = slideMin;
            }
            if (slidePositionTarget > slideMax) {
                slidePositionTarget = slideMax;
            }
            if (slidePositionTarget > 200.0) {
                scalar = 1.0 - Math.sqrt(slidePositionTarget/slideMax)/1.25;
            }
        //}

        telemetry.addData("Slide target: ", slidePositionTarget);
        telemetry.addData("Error RS", "Error RS: " + (slidePositionTarget - rs.getCurrentPosition()));
        telemetry.addData("Error LS", "Error LS: " + (slidePositionTarget - ls.getCurrentPosition()));

        ls.setPower(slidesff + SlidesPID.calculatePower(slidePositionTarget, ls.getCurrentPosition(), timegap));
        rs.setPower(slidesff + SlidesPID.calculatePower(slidePositionTarget, rs.getCurrentPosition(), timegap));


        if (gamepad1.left_trigger > 0.5 && scalar > 0.3){
            scalar = 0.3;
        }

        double preRF = r*Math.sqrt(scalar) + y*Math.cbrt(scalar) + x*scalar;
        double preLF = r*Math.sqrt(scalar) + y*Math.cbrt(scalar) - x*scalar;
        double preRB = r*Math.sqrt(scalar) - y*Math.cbrt(scalar) + x*scalar;
        double preLB = r*Math.sqrt(scalar) - y*Math.cbrt(scalar) - x*scalar;

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
        if (gamepad1.x) {
            //armRight.setPosition(0.0);
            armLeft.setPosition(armOutPos);
        }

        //arm swings in
        if (gamepad1.b) {
            //armRight.setPosition(1.0);
            armLeft.setPosition(armInPos);
        }

        //plunger open
        if (gamepad1.y) {
            pRight.setPosition(plungerGrabPos);
            pLeft.setPosition(plungerGrabPos);
        }

        //plunger close
        if (gamepad1.a) {
            pRight.setPosition(plungerReleasePos);
            pLeft.setPosition(plungerReleasePos);
        }

        if (gamepad1.dpad_right) {
            slidePositionTarget = slideSavedPosition;
        }

        if (gamepad1.dpad_down) {
            if (!dpadDownPressed) {
                slideSavedPosition = slidePositionTarget;
                slidePositionTarget = slideMin;
            }
            dpadDownPressed = true;
        } else {
            dpadDownPressed = false;
        }

        //if (gamepad1.dpad_up) {
        //    slidePositionTarget = slideMax;
        //}

        //intake
        if (gamepad1.left_bumper && (gamepad1.right_bumper || gamepad1.dpad_left)) {
            intake.setPower(-0.25); //reverse
        } else if (gamepad1.right_trigger > 0.2 && (gamepad1.right_bumper || gamepad1.dpad_left)) {
            intake.setPower(0.90); //forward
            slidePositionTarget = 150.0;
        } else {
            intake.setPower(0.0);
        }

        //swinging intake out
        if (gamepad1.right_bumper) {
            intakeRight.setPosition(intakeDownPos);
            intakeLeft.setPosition(intakeDownPos);
        } else {
            intakeRight.setPosition(intakeUpPos);
            intakeLeft.setPosition(intakeUpPos);
        }

        //stack position intake
        if (gamepad1.dpad_left) {
            intakeRight.setPosition(intakeStackPos);
            intakeLeft.setPosition(intakeStackPos);
        }

        // drone launcher
        if (gamepad1.back) {
            droneLauncher.setPosition(1.0);
        }
        //droneLauncher.setPosition(dronePos2);

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
        telemetry.update();

        packet.put("slides target", slidePositionTarget);
        packet.put("slides position rs", rs.getCurrentPosition());
        packet.put("slides position ls", ls.getCurrentPosition());
        packet.put("slides power rs", rs.getPower());
        packet.put("slides power ls", ls.getPower());
        packet.put("slides limit pressed", slidesLimit.isPressed());
        packet.put("slide saved position", slideSavedPosition);
        packet.put("scalar", scalar);
        dashboard.sendTelemetryPacket(packet);
    }

    /*public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error-lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double power = (error * kP) + (derivative * kD) + (integralSum*kI);
        return Math.tanh(error);
    }*/
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
