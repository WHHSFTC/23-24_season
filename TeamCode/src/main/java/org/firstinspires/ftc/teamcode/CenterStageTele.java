package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    double slidePositionTarget = 0.0;

    public static double dronePos1 = 0.2;
    public static double dronePos2 = 0.6;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double timegap;


    //Servo armRight;
    Servo armLeft;
    Servo pRight;
    Servo pLeft;
    Servo intakeRight;
    Servo intakeLeft;
    Servo droneLauncher;

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


        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);
        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        pRight.scaleRange(0.72, 0.77);
        pLeft.scaleRange(0.60,0.67);

        pRight.setPosition(1.0);
        pLeft.setPosition(1.0);

        droneLauncher.scaleRange(dronePos1, dronePos2);

        /*rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE); */
    }

    @Override
    public void loop(){
        //find timegapr
        /*timegap = timer.milliseconds();
        timer.reset();*/

        double y = -gamepad1.left_stick_x; //verticals
        double x = -gamepad1.left_stick_y*1.3; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation

        /*if (gamepad1.back) {
            ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }else{
            ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }*/

        /*if (gamepad2.left_stick_y > 0.0) {
            slidePositionTarget -= (15 * gamepad2.left_stick_y);
            if(slidePositionTarget < -170){
                slidePositionTarget = -170;
            }}

        if (gamepad2.left_stick_y < 0.0){
                slidePositionTarget += (-10 * gamepad2.left_stick_y);
                if(slidePositionTarget > 2300){
                slidePositionTarget = 2300;
            }
            }

            telemetry.addData("Slide target: ", slidePositionTarget);
            telemetry.addData("Error RS", "Error LS: " + (slidePositionTarget - rs.getCurrentPosition()));
            telemetry.addData("Error LS", "Error LS " + (slidePositionTarget - ls.getCurrentPosition()));

            ls.setPower(SlidesPID.calculatePower(slidePositionTarget, ls.getCurrentPosition(), timegap));
            rs.setPower(SlidesPID.calculatePower(slidePositionTarget, rs.getCurrentPosition(), timegap));
        */

        if ((gamepad2.left_stick_y*1.2) < 0){
            ls.setTargetPosition(2200);
            rs.setTargetPosition(2200);

            ls.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rs.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ls.setPower(0.5);
            rs.setPower(0.5);
        } else if(gamepad2.left_stick_y*1.2 > 0){
            ls.setTargetPosition(0);
            rs.setTargetPosition(0);

            ls.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rs.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ls.setPower(0.4);
            rs.setPower(0.4);
        }else{
            ls.setTargetPosition(ls.getCurrentPosition());
            rs.setTargetPosition(ls.getCurrentPosition());

            ls.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rs.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ls.setPower(0.5);
            rs.setPower(0.5);
        }

        boolean turtle = false;
        if(gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5){
            turtle = true;
        }

        double scalar;
        if(turtle){
            scalar = 0.40;
        }else{
            scalar = 1.0;
        }
        double preRF = (r+(y+x))*scalar;
        double preLF = (r+(y-x))*scalar;
        double preRB = (r+(-y+x))*scalar;
        double preLB = (r+(-y-x))*scalar;

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
        if(gamepad2.x){
            //armRight.setPosition(0.0);
            armLeft.setPosition(0.0);
        }

        //arm swings in
        if(gamepad2.b){
            //armRight.setPosition(1.0);
            armLeft.setPosition(1.0);
        }

        //plunger open
        if(gamepad2.y){
            pRight.setPosition(0.0);
            pLeft.setPosition(0.0);
        }

        //plunger close
        if(gamepad2.a){
            pRight.setPosition(1.0);
            pLeft.setPosition(1.0);
        }

        //intake
        if(gamepad2.left_trigger > 0.2){
            intake.setPower(-0.95);
        } else {
            intake.setPower(0.0);
        }

        if(gamepad2.right_trigger > 0.2){
            intake.setPower(0.98);
        } else {
            intake.setPower(0.0);
        }

        //swinging intake to init position
        if(gamepad2.dpad_up){
            intakeRight.setPosition(1.0);
            intakeLeft.setPosition(1.0);
        }

        //swinging intake out
        if(gamepad2.dpad_down){
            intakeRight.setPosition(0.0);
            intakeLeft.setPosition(0.0);
        }

        //stack position intake
        if(gamepad2.dpad_right){
            intakeRight.setPosition(0.25);
            intakeLeft.setPosition(0.25);
        }

        // drone launcher
        if(gamepad2.back){
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

        /*packet.put("slides target", slidePositionTarget);
        packet.put("slides position rs", "current rs pos: " + rs.getCurrentPosition());
        packet.put("slides position ls", "current ls pos: " + ls.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);*/
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
