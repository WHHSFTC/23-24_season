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
    public double delay;
    private Servo mechanism;
    private DcMotor mot;
    private double initial;
    private double target;
    public ElapsedTime delayTimer; //TODO: make private again
    public static ArrayList<DelaysAndAutoms> allDelays = new ArrayList<DelaysAndAutoms>();

    public DelaysAndAutoms(double delay, Servo mechanism, double initial, double target) {
        delayTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.delay = delay;
        this.mechanism = mechanism;
        this.initial = initial;
        this.target = target;
        allDelays.add(this);
    }

    public DelaysAndAutoms(double delay, DcMotor mot, double initial, double target) {
        delayTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.target = target;
        this.mot = mot;
        this.delay = delay;
        this.initial = initial;
        allDelays.add(this);
    }

    public DelaysAndAutoms(double delay, double initial, double target) {
        delayTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.target = target;
        this.delay = delay;
        this.initial = initial;
        allDelays.add(this);
    }

    public static void updateDelays() {
        for (int i = 0; i < allDelays.size(); i++) {
            if (allDelays.get(i).mechanism != null) {
                if (allDelays.get(i).delayTimer.milliseconds() < allDelays.get(i).delay) {
                    //allDelays.get(i).mechanism.setPosition(allDelays.get(i).initial);
                } else {
                    allDelays.get(i).mechanism.setPosition(allDelays.get(i).target);
                    allDelays.remove(i);
                }
            } else if (allDelays.get(i).mot != null) {
                if (allDelays.get(i).delayTimer.milliseconds() < allDelays.get(i).delay) {
                    //allDelays.get(i).mot.setPower(allDelays.get(i).initial);
                } else {
                    allDelays.get(i).mot.setPower(allDelays.get(i).target);
                    allDelays.remove(i);
                }
            }
        }
    }

    public double updateVariable(){
        if (delayTimer.milliseconds() < delay){
            return initial;
        }
        else{
            allDelays.remove(this);
            return target;
        }
    }
}
    