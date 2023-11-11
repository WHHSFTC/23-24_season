
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;




@Autonomous (preselectTeleOp = "CenterStageTele")
public class ScoreRed extends OpMode{

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

    private ElapsedTime runtime = new ElapsedTime();

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

        runtime.reset();
    }

    @Override
    public void start(){
        runtime.reset();
    }
    @Override
    public void loop(){

    }

    public void stop(){

    }
}