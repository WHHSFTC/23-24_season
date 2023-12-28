package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DelaysAndAutoms {

    public static void delayServo(double delay, Servo mechanism, double initialPos, double finalPos){
        ElapsedTime timerServo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timerServo.reset();

        boolean delayDone = timerServo.time() >= delay;
        while (!delayDone) {
            mechanism.setPosition(initialPos);
        }
        mechanism.setPosition(finalPos);
    }

    public static void delayMotor(double delay, DcMotor motor, double initialPower, double finalPower){
        ElapsedTime timerMotor = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timerMotor.reset();

        boolean delayDone = timerMotor.time() >= delay;
        while (!delayDone) {
            motor.setPower(initialPower);
        }
        motor.setPower(finalPower);
    }
}