package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (preselectTeleOp = "CenterStageTele")
public class ParkAutoRightRed extends OpMode{

    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;

    Servo intakeRight;
    Servo intakeLeft;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init(){
        rf = hardwareMap.get(DcMotor.class, "motorRF");
        lf = hardwareMap.get(DcMotor.class, "motorLF");
        rb = hardwareMap.get(DcMotor.class, "motorRB");
        lb = hardwareMap.get(DcMotor.class, "motorLB");

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");

        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);

        intakeRight.setDirection(Servo.Direction.REVERSE);

        intakeLeft.scaleRange(0.0, 0.55);
        intakeRight.scaleRange(0.22, 1);

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
        if(runtime.seconds() < 0.3){
            rf.setPower(-0.5);
            lf.setPower(-0.5);
            rb.setPower(0.5);
            lb.setPower(0.5);
        } else if(runtime.seconds() >= 0.3  && runtime.seconds() < 1.8){
            rf.setPower(-0.5);
            lf.setPower(0.5);
            rb.setPower(-0.5);
            lb.setPower(0.5);
        } else {
            rf.setPower(0);
            lf.setPower(0);
            rb.setPower(0);
            lb.setPower(0);
        }
    }

    @Override
    public void stop(){
        super.stop();
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }
}

