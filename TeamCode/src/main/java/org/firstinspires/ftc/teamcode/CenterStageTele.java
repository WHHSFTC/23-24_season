package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class CenterStageTele extends OpMode{

    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;
    DcMotor intake;

    double slidePositionTarget = 0.0;
    // Gains for slides, to be tuned
    double kP = 0.0;
    double kI = 0;
    double kD = 0;
    double integralSum = 0.0;
    private double lastError = 0.0;
    ElapsedTime timer = new ElapsedTime();
    DcMotor ls;
    DcMotor rs;

    Servo armRight;
    Servo armLeft;
    Servo pRight;
    Servo pLeft;
    Servo intakeRight;
    Servo intakeLeft;
    @Override
    public void init(){
        rf = hardwareMap.get(DcMotor.class, "motorRF");
        lf = hardwareMap.get(DcMotor.class, "motorLF");
        rb = hardwareMap.get(DcMotor.class, "motorRB");
        lb = hardwareMap.get(DcMotor.class, "motorLB");
        intake = hardwareMap.get(DcMotor.class, "motorIntake");
        ls = hardwareMap.get(DcMotor.class, "motorLS");
        rs = hardwareMap.get(DcMotor.class, "motorRS");

        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        pRight = hardwareMap.get(Servo.class, "plungerRight");
        pLeft = hardwareMap.get(Servo.class, "plungerLeft");

        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");

        intakeRight.setDirection(Servo.Direction.REVERSE);

        intakeLeft.scaleRange(0.0, 0.55);
        intakeRight.scaleRange(0.22, 1);

        intakeLeft.setPosition(1.0);
        intakeRight.setPosition(1.0);

        armLeft.scaleRange(0.0, 0.245);
        armRight.scaleRange(0.0, 0.245);

        armLeft.setPosition(0.0);

        pRight.scaleRange(0.72, 0.77);
        pLeft.scaleRange(0.60,0.67);

        pRight.setPosition(1.0);
        pLeft.setPosition(1.0);

        /*rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE); */
    }

    @Override
    public void loop(){
        double y = -gamepad1.left_stick_x; //verticals
        double x = -gamepad1.left_stick_y*1.3; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation

        //double slidesScalar = 0.01; // factor for motor power
        //slidePositionTarget += Math.max(0, slidesScalar * gamepad2.left_stick_y);
        telemetry.addData("Slide target: ", slidePositionTarget);

        //ls.setPower(PIDControl(slidePositionTarget, ls.getCurrentPosition()));
        //rs.setPower(PIDControl(slidePositionTarget, rs.getCurrentPosition()));

        ls.setPower(gamepad2.left_stick_y*1.2);
        rs.setPower(gamepad2.left_stick_y*1.2);

        if ((gamepad2.left_stick_y*1.2) < 0){
            ls.setTargetPosition(1600);
            rs.setTargetPosition(1600);
        } else {
            ls.setTargetPosition(0);
            rs.setTargetPosition(0);
        }

        ls.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rs.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean turtle = false;
        if(gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5){
            turtle = true;
        }

        double scalar;
        if(turtle){
            scalar = 0.25;
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
            armRight.setPosition(0.0);
            armLeft.setPosition(0.0);
        }

        //arm swings in
        if(gamepad2.b){
            armRight.setPosition(1.0);
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
            intake.setPower(0.95);
        } else {
            intake.setPower(0.0);
        }

        if(gamepad2.right_trigger > 0.2){
            intake.setPower(-0.95);
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
            intakeRight.setPosition(0.2);
            intakeLeft.setPosition(0.2);
        }

        telemetry.addData("rf", postRF);
        telemetry.addData("lf", postLF);
        telemetry.addData("rb", postRB);
        telemetry.addData("lb", postLB);

        telemetry.addData("intake", intake.getPower());
        telemetry.addData("armRight", "Position: " + armRight.getPosition());
        telemetry.addData("armLeft", "Position: " + armLeft.getPosition());
        telemetry.addData("pRight", "Position: " + pRight.getPosition());
        telemetry.addData("pLeft", "Position: " + pLeft.getPosition());
        telemetry.addData("intakeRight", "Position: " + intakeRight.getPosition());
        telemetry.addData("intakeLeft", "Position: " + intakeLeft.getPosition());
        telemetry.addData("rs", "Power: " + rs.getPower() + "Position: " + rs.getCurrentPosition());
        telemetry.addData("ls", "Power: " + ls.getPower() + "Position: " + ls.getCurrentPosition());
        telemetry.update();
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
