package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.GraphUtils;

import java.util.ArrayList;

public class DelaysAndAutoms extends drivetraintele {
    private double delay;
    private Servo mechanism;
    private DcMotor mot;
    private Double value;
    private double initial;
    private double target;
    public ElapsedTime delayTimer; //TODO: make private again
    public static ArrayList<DelaysAndAutoms> allDelays = new ArrayList<DelaysAndAutoms>();

    public DelaysAndAutoms(double delay, Servo mechanism, double initial, double target) {
        delayTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.target = target;
        this.mechanism = mechanism;
        this.initial = initial;
        this.target = target;
    }

    public DelaysAndAutoms(double delay, DcMotor mot, double initial, double target) {
        delayTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.target = target;
        this.mot = mot;
        this.delay = delay;
        this.initial = initial;
        allDelays.add(this);
    }

    public DelaysAndAutoms(double delay, Double value, double initial, double target) {
        delayTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.target = target;
        this.value = value;
        this.delay = delay;
        this.initial = initial;
        allDelays.add(this);
    }

    public static void updateDelays() {
        for (DelaysAndAutoms it : allDelays) {
            if (it.mechanism != null) {
                if (it.delayTimer.milliseconds() < it.delay) {
                    it.mechanism.setPosition(it.initial);
                } else {
                    it.mechanism.setPosition(it.target);
                    allDelays.remove(it);
                }
            } else if (it.mot != null) {
                if (it.delayTimer.milliseconds() < it.delay) {
                    it.mot.setPower(it.initial);
                } else {
                    it.mot.setPower(it.target);
                    allDelays.remove(it);
                }
            } else {
                if (it.delayTimer.milliseconds() < it.delay) {
                    it.value = it.initial;
                } else {
                    it.value = it.target;
                    allDelays.remove(it);
                }
            }
        }
    }
}
    