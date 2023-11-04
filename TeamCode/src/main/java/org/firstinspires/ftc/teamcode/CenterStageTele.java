package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class CenterStageTele extends OpMode{

    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;
    DcMotor intake;

    double slidePositionTarget = 0.0;
    double previousLeftError = 0.0;
    double previousRightError = 0.0;
    double leftInt = 0.0;
    double rightInt = 0.0;

    // Gains for slides, to be tuned
    double kP = 1.0;
    double kI = 1.0;
    double kD = 1.0;

    DcMotor ls;
    DcMotor rs;
    /*
    Servo armRight;
    Servo armLeft;
    Servo pRight;
    Servo pLeft;
    Servo intakeRight;
    Servo intakeLeft; */
    @Override
    public void init(){
        rf = hardwareMap.get(DcMotor.class, "motorRF");
        lf = hardwareMap.get(DcMotor.class, "motorLF");
        rb = hardwareMap.get(DcMotor.class, "motorRB");
        lb = hardwareMap.get(DcMotor.class, "motorLB");
        intake = hardwareMap.get(DcMotor.class, "motorIntake");
        ls = hardwareMap.get(DcMotor.class, "motorLS");
        rs = hardwareMap.get(DcMotor.class, "motorRS");
/*
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        pRight = hardwareMap.get(Servo.class, "plungerRight");
        pLeft = hardwareMap.get(Servo.class, "plungerLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");

        intakeLeft.setPosition(0.0);
        intakeRight.setPosition(0.0); */

//        rf.setDirection(DcMotorSimple.Direction.REVERSE);
//        rb.setDirection(DcMotorSimple.Direction.REVERSE);
//        lb.setDirection(DcMotorSimple.Direction.REVERSE);
//        lf.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){

        double y = -gamepad1.left_stick_x; //vertical
        double x = -gamepad1.left_stick_y*1.1; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation

        double slidesScalar = 1.0; // factor for motor power
        slidePositionTarget += slidesScalar * gamepad2.right_stick_y;
        telemetry.addData("Slide target: ", slidePositionTarget);

        // this calculates error for both slides and assigns a value to the duration of each loop cycle
        double leftError = slidePositionTarget - 0.0; //<-- PLACEHOLDER, THIS SHOULD BE KNOWN FROM AN ENCODER, also could be the opposite of this
        double rightError = slidePositionTarget - 0.0; //<-- PLACEHOLDER, THIS SHOULD BE KNOWN FROM AN ENCODER, also could be the opposite of this
        double loopDuration = 1.0; //<-- PLACEHOLDER, I DON'T KNOW HOW WE GET THIS

        // this should calculate the P I and D values based on error, but should be checked
        double propL =  leftError;
        double derivL = leftError - previousLeftError / loopDuration;
        leftInt+=leftError * loopDuration;
        double propR =  rightError;
        double derivR = rightError - previousRightError / loopDuration;
        rightInt+=rightError * loopDuration;

        // power slide motors
        ls.setPower(kP*propL + kD*derivL + kI*leftInt);
        rs.setPower(kP*propR + kD*derivR + kI*rightInt);


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

       /* if(gamepad2.b){
            armRight.setPosition(0.0);
            armLeft.setPosition(0.0);
        }

        if(gamepad2.x){
            armRight.setPosition(0.4);
            armLeft.setPosition(0.4);
        }
        */
        intake.setPower((gamepad2.right_bumper)? -0.95 : 0);
        intake.setPower((gamepad2.left_bumper)? 0.6 : 0);

        telemetry.addData("rf", postRF);
        telemetry.addData("lf", postLF);
        telemetry.addData("rb", postRB);
        telemetry.addData("lb", postLB);

        telemetry.addData("intake", intake.getPower());
       /* telemetry.addData("armRight", armRight.getPosition());
        telemetry.addData("armLeft", armLeft.getPosition());
         */
        telemetry.update();
    }

    @Override
    public void stop(){
        super.stop();
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        intake.setPower(0);
    }
}
