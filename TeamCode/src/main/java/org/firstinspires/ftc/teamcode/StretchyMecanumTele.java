package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class StretchyMecanumTele extends CenterStageTeleProper {

    public static double strafeScalar = 1.1;
    @Override
    public void childLoop(){

        double ly = -gamepad1.left_stick_x;
        double lx = -gamepad1.left_stick_y * strafeScalar;
        double ry = -gamepad1.right_stick_x;
        double rx = -gamepad1.right_stick_y * strafeScalar;

        double preRF = ry - rx;
        double preRB = ry + rx;
        double preLF = -ly - lx;
        double preLB = -ly + rx;

        double max = Math.max(Math.max(Math.max(Math.max(preRF,preRB), preLB), preLF), 1);

        double postRF = preRF/max;
        double postRB = preRB/max;
        double postLF = preLF/max;
        double postLB = preLB/max;

        rf.setPower(postRF);
        rb.setPower(postRB);
        lf.setPower(postLF);
        lb.setPower(postLB);

        telemetry.addData("rf", postRF);
        telemetry.addData("rb", postRB);
        telemetry.addData("lf", postLF);
        telemetry.addData("lb", postLB);
        telemetry.addData("rx", rx);
        telemetry.addData("ry", ry);
        telemetry.addData("lx", lx);
        telemetry.addData("ly", ly);
        telemetry.update();
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
