package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public abstract class CenterStageAuto extends CenterStageOpMode {

    enum State {
        SCANNING,
        PURPLE,
        YELLOW,
        CYCLE,
        PARK
    }

    State currentState = State.PURPLE;
    int elementPosition; //0 1 or 2 -- initialize in start()

    @Override
    public void init() {
        super.init();
    }

    public void start() {
        //vision
    }

    @Override
    public void loop() {
        super.loop();
        switch(currentState) {

        }
    }
}
