package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DelaysAndAutoms extends drivetraintele {
    private double delay;
    private Servo mechanism;
    private DcMotor mot;
    private double initialPos;
    private double finalPos;
    private double initialPower;
    private double finalPower;
    private ElapsedTime delayTimer;

    public DelaysAndAutoms(double delay, Servo mechanism, double initialPos, double finalPos) {
        delayTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.finalPos = finalPos;
        this.mechanism = mechanism;
        this.initialPos = initialPos;
        this.finalPos = finalPos;
    }

    public DelaysAndAutoms(double delay, DcMotor mot, double initialPower, double finalPower) {
        delayTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.finalPower = finalPower;
        this.mot = mot;
        this.delay = delay;
        this.initialPower = initialPower;
    }

    public void delay() {

        if (mechanism != null) {
            

            if (delayTimer.time() < delay) {
                mechanism.setPosition(initialPos);
            } else {
                mechanism.setPosition(finalPos);
            }
        } else {
            if (delayTimer.time() < delay) {
                mot.setPower(initialPower);
            } else {
                mot.setPower(finalPower);
            }
        }
    }
}
    