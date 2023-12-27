package org.firstinspires.ftc.teamcode.subsystems;

public class Output {
    public enum OutputState {
        INIT(0.0, 1.0, false),
        INTAKE(150.0, 1.0, false),
        GRAB(0.0, 1.0, true),
        LIFT(300.0, 1.0, true),
        OUTPUT(300.0, 0.0, true);

        public double slidesHeight;
        public double armPosition;
        public boolean plungersOpen;

        private OutputState(double slidesHeight, double armPosition, boolean plungersOpen) {
            this.slidesHeight = slidesHeight;
            this.armPosition = armPosition;
            this.plungersOpen = plungersOpen;
        }
    }
    OutputState state = OutputState.OUTPUT;

}
