
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (preselectTeleOp = "CenterStageTele")
public class ScoreBlue extends OpMode{

    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;
    DcMotor ls;
    DcMotor rs;

    Servo intakeRight;
    Servo intakeLeft;
    Servo pRight;
    Servo pLeft;
    Servo armRight;
    Servo armLeft;

    //private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init(){
        rf = hardwareMap.get(DcMotor.class, "motorRF");
        lf = hardwareMap.get(DcMotor.class, "motorLF");
        rb = hardwareMap.get(DcMotor.class, "motorRB");
        lb = hardwareMap.get(DcMotor.class, "motorLB");
        ls = hardwareMap.get(DcMotor.class, "motorLS");
        rs = hardwareMap.get(DcMotor.class, "motorRS");

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        pRight = hardwareMap.get(Servo.class, "plungerRight");
        pLeft = hardwareMap.get(Servo.class, "plungerLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        armLeft = hardwareMap.get(Servo.class, "armLeft");

        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);

        rf.setPower(0.0);
        lf.setPower(0.0);
        rb.setPower(0.0);
        lb.setPower(0.0);

        intakeRight.setDirection(Servo.Direction.REVERSE);

        intakeLeft.scaleRange(0.0, 0.55);
        intakeRight.scaleRange(0.22, 1);

        armLeft.scaleRange(0.0, 0.245);
        armRight.scaleRange(0.0, 0.245);

        pRight.scaleRange(0.72, 0.77);
        pLeft.scaleRange(0.60,0.67);

        intakeLeft.setPosition(1.0);
        intakeRight.setPosition(1.0);

        //runtime.reset();
    }

    @Override
    public void start(){


        //runtime.reset();
    }
    @Override
    public void loop(){

        /*if(runtime.seconds() < 1.0 ){
            rf.setPower(0.5);
            lf.setPower(0.5);
            rb.setPower(-0.5);
            lb.setPower(-0.5);
        } else if(runtime.seconds() >= 1.0  && runtime.seconds() < 2.0){
            rf.setPower(-0.5);
            lf.setPower(0.5);
            rb.setPower(-0.5);
            lb.setPower(0.5);
        } else if (runtime.seconds() >= 2.0 && runtime.seconds() < 3.0){
            pRight.setPosition(0.0);
            pLeft.setPosition(0.0);
        }
        else if(runtime.seconds() >= 3.0 && runtime.seconds() < 6.0){
            rs.setTargetPosition(800);
            ls.setTargetPosition(800);
            ls.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rs.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ls.setPower(0.4);
            rs.setPower(0.4);
        }
        else if(runtime.seconds() >= 6.0 && runtime.seconds() < 8.0){
            armRight.setPosition(0.0);
            armLeft.setPosition(0.0);
        }
        else if(runtime.seconds() >= 8.0 && runtime.seconds() < 8.2){
            rf.setPower(-0.2);
            lf.setPower(0.2);
            rb.setPower(-0.2);
            lb.setPower(0.2);
        }
        else if(runtime.seconds() >= 8.2 && runtime.seconds() < 9.5){
            pRight.setPosition(1.0);
            pLeft.setPosition(1.0);
        }
        else if(runtime.seconds() >= 9.5 && runtime.seconds() < 10.5){
            rf.setPower(0.1);
            rb.setPower(0.1);
            lf.setPower(-0.1);
            lb.setPower(-0.1);
        }
        else if(runtime.seconds() >= 10.5 && runtime.seconds() < 12.0){
            rs.setTargetPosition(0);
            ls.setTargetPosition(0);
            ls.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rs.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ls.setPower(0.4);
            rs.setPower(0.4);
        }
        else if(runtime.seconds() >= 12 && runtime.seconds() < 12.7){
            rf.setPower(-0.5);
            rb.setPower(0.5);
            lf.setPower(-0.5);
            lb.setPower(0.5);
        }
        else if(runtime.seconds() >= 12.7 && runtime.seconds() < 13.1){
            rf.setPower(-0.5);
            lf.setPower(0.5);
            rb.setPower(-0.5);
            lb.setPower(0.5);
        }
        else{
            rf.setPower(0.0);
            lf.setPower(0.0);
            rb.setPower(0.0);
            lb.setPower(0.0);
            ls.setPower(0.0);
            rs.setPower(0.0);
        }*/
    }
    public void stop(){
        super.stop();
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        ls.setPower(0);
        rs.setPower(0);
    }
}