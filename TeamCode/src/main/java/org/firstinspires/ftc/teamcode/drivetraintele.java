package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class drivetraintele extends CenterStageTeleProper {

    @Override
    public void childLoop(){
        double y = -gamepad1.left_stick_x; //verticals
        double x = -gamepad1.left_stick_y*1.1; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation

        double preRF = 1.3*r* + y + x;
        double preLF = 1.3*r + y - x;
        double preRB = 1.3*r - y + x;
        double preLB = 1.3*r - y - x;

        double max = Math.max(Math.max(Math.max(Math.max(preRF,preRB), preLB), preLF), 1);

        rf.setPower(preRF/max);
        lf.setPower(preLF/max);
        rb.setPower(preRB/max);
        lb.setPower(preLB/max);

        double postRF = preRF/max;
        double postLF = preLF/max;
        double postRB = preRB/max;
        double postLB = preLB/max;

        telemetry.addData("rf", postRF);
        telemetry.addData("lf", postLF);
        telemetry.addData("rb", postRB);
        telemetry.addData("lb", postLB);
        telemetry.update();

        //controls

        if (gamepad1.back){

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
