package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DelaysAndAutoms extends drivetraintele{

    public static void delayServo(double delay, Servo mechanism, double initialPos, double finalPos){
        ElapsedTime timerServo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timerServo.reset();

        while (timerServo.time() < delay) {
            mechanism.setPosition(initialPos);
        }
        mechanism.setPosition(finalPos);
    }

    public static void delayMotor(double delay, DcMotor motor, double initialPower, double finalPower) {
        ElapsedTime timerMotor = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timerMotor.reset();

        while (timerMotor.time() < delay) {
            motor.setPower(initialPower);
        }
        motor.setPower(finalPower);
    }
}