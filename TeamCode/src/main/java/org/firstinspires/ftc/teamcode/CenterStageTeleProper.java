package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DelaysAndAutoms.updateDelays;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class CenterStageTeleProper extends CenterStageOpMode{

    //public static MultipleTelemetry dashTelemetry = new MultipleTelemetry();

    public static double slidesff = 0.0;
    public static double slideTargetGain = 300.0;
    public static double slideMin = 0.0;
    public static double slideMax = 2700.0;
    double distancePower;
    double anglePower;
    boolean slidesPressed = true;

    double targetYaw = 0.0;
    double timeGap;
    boolean intakeOnGround;
    boolean plungerLClosed;
    boolean plungerRClosed;
    boolean zeroing = false;
    boolean fieldCentric = false;

    @Override
    public void init(){
        super.init();

        //droneLauncher.scaleRange(dronePos1, dronePos2);
        //droneLauncher.setPosition(1.0);
        plungerLClosed = true;
        plungerRClosed = true;

        /*rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE); */

        imu.resetYaw();
        intakeRight.setPosition(intakeDownPos);
        intakeLeft.setPosition(intakeDownPos);
        intakeOnGround = true;
    }

    @Override
    public void childLoop(){
        slidesPidRight.update(rs.getCurrentPosition(), timeGap);
        slidesPidLeft.update(ls.getCurrentPosition(), timeGap);
        //find timeGap
        timeGap = timer.milliseconds();
        timer.reset();

        double scalar = 1.0;

        double y = -gamepad1.left_stick_x; //verticals
        double x = -gamepad1.left_stick_y*1.1; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation

        if (fieldCentric) {
            double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double tempX = x;
            double tempY = y;
            x = tempX * Math.cos(-angle) - tempY * Math.sin(-angle);
            y = tempX * Math.sin(-angle) + tempY * Math.cos(-angle);
        }


        if (gamepad1.dpad_up) {
            fieldCentric = true;
        }
        if (gamepad1.dpad_down) {
            fieldCentric = false;
        }
        telemetry.addData("field centric:", fieldCentric);

        if(gamepad1.a){
            distancePower = DSpid.distancePID(rightDS.getDistance(DistanceUnit.INCH), leftDS.getDistance(DistanceUnit.INCH), timeGap, distBackdrop);
            anglePower = DSpid.anglePID(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), timeGap, Math.toRadians(0));
            x = x>0.55 ? x : -distancePower;
            r = -anglePower;
            scalar = 0.25;
        }

        //heading lock
        if(gamepad1.b){
            if(!gamepad1prev.b) {
                targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }
            anglePower = DSpid.anglePID(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), timeGap, targetYaw);
            r = -anglePower;
        }

        //slides
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (slidesLimit.isPressed()) {
            if (!slidesPressed) {
                ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            slidesPressed = true;
        } else {
            slidesPressed = false;
        }

        if (Math.abs(gamepad2.left_stick_y) > 0.01) {
            zeroing = false;

            if (Math.abs(gamepad2.left_stick_y) > 0.01) {slidePositionTarget -= slideTargetGain * gamepad2.left_stick_y;}

            if (slidePositionTarget < slideMin) {
                slidePositionTarget = slideMin;
            }
            if (slidePositionTarget > slideMax) {
                slidePositionTarget = slideMax;
            }
            if (slidePositionTarget > 600.0) {
                scalar = 1.0 - (Math.sqrt(slidePositionTarget/slideMax)/1.25);
            }
        }

        telemetry.addData("Slide target: ", slidePositionTarget);
        telemetry.addData("Error RS", "Error RS: " + (slidePositionTarget - rs.getCurrentPosition()));
        telemetry.addData("Error LS", "Error LS: " + (slidePositionTarget - ls.getCurrentPosition()));

        if (!zeroing) {
            rs.setPower(slidesPidRight.calculatePower(slidePositionTarget));
            ls.setPower(slidesPidLeft.calculatePower(slidePositionTarget));
        }

        if (gamepad1.left_bumper && scalar > 0.3){
            scalar = 0.3;
        }

        double preRF = 1.3*r*Math.sqrt(scalar) + y*Math.cbrt(scalar) + x*scalar;
        double preLF = 1.3*r*Math.sqrt(scalar) + y*Math.cbrt(scalar) - x*scalar;
        double preRB = 1.3*r*Math.sqrt(scalar) - y*Math.cbrt(scalar) + x*scalar;
        double preLB = 1.3*r*Math.sqrt(scalar) - y*Math.cbrt(scalar) - x*scalar;


        double max = Math.max(Math.max(Math.max(Math.max(preRF,preRB), preLB), preLF), 1);


        rf.setPower(preRF/max);
        lf.setPower(preLF/max);
        rb.setPower(preRB/max);
        lb.setPower(preLB/max);

        //arm swings out
        if (gamepad2.x) {
            //armRight.setPosition(0.0);
            armLeft.setPosition(armOutPos);
        }

        //arm swings in
        if (gamepad2.b) {
            //armRight.setPosition(1.0);
            armLeft.setPosition(armInPos);
        }

        //plunger open
        if (gamepad2.y) {
            pRight.setPosition(plungerGrabPos);
            pLeft.setPosition(plungerGrabPos);
            plungerLClosed = false;
            plungerRClosed = false;
        }

        //plunger close
        if (gamepad2.a) {
            pRight.setPosition(plungerReleasePos);
            pLeft.setPosition(plungerReleasePos);
            plungerLClosed = true;
            plungerRClosed = true;
        }

        if (gamepad2.dpad_right && !zeroing) {
            slidePositionTarget = slideSavedPosition;
        }

        if (gamepad2.dpad_down && !zeroing && (pLeft.getPosition() > 0.9 && pRight.getPosition() > 0.9)) {
            if (!gamepad2prev.dpad_down) {
                slideSavedPosition = slidePositionTarget;
                slidePositionTarget = slideMin;
                zeroing = true;
            }
        }

        //intake spinning
        if (gamepad1.left_trigger > 0.2) {
            intake.setPower(-0.3 * gamepad1.left_trigger); //reverse
        } else if (gamepad1.right_trigger > 0.2) {
            intake.setPower(gamepad1.right_trigger*0.5); //forward
        } else {
            intake.setPower(0.0);
        }

        //intake swinging out and swinging in
        if(gamepad1.right_bumper && !gamepad1prev.right_bumper){
            if(intakeOnGround){
                intakeRight.setPosition(intakeUpPos);
                intakeLeft.setPosition(intakeUpPos);
                intakeOnGround = false;
            }else{
                intakeRight.setPosition(intakeDownPos);
                intakeLeft.setPosition(intakeDownPos);
                intakeOnGround = true;
            }
        }

        if(gamepad2.right_trigger > 0.05){
            conveyor.setPower(-0.9);
        }
        else if(gamepad2.left_trigger > 0.05){
            conveyor.setPower(0.9);
        }else{
            conveyor.setPower(0.0);
        }

        //stack position intake
        if (gamepad1.dpad_right) {
            if(!intakeOnGround){
                intakeOnGround = true;
            }
            intakeRight.setPosition(intakeStackPos);
            intakeLeft.setPosition(intakeStackPos);
        }

        //reset imu
        if(gamepad1.back){
            imu.resetYaw();
        }

        //manual reset for slides
        if (gamepad2.start) {
            zeroing = true;
        }

        if (zeroing) {
            slidePositionTarget = 0.0;
            rs.setPower(-0.3);
            ls.setPower(-0.3);
        }
        if (slidesLimit.isPressed()) {
            zeroing = false;
        }

        //hammer
        if(gamepad1.y){
            intakeLeft.setPosition(0.8);
            intakeRight.setPosition(0.8);
            /*DelaysAndAutoms hammerLeft = */new DelaysAndAutoms(200, intakeLeft, 0.8, intakeDownPos);
            /*DelaysAndAutoms hammerRight = */new DelaysAndAutoms(200, intakeRight, 0.8, intakeDownPos);
        }

        // drone launcher
        if (gamepad2.back) {
            droneLauncher.setPosition(dronePos1);
        } else {
            droneLauncher.setPosition(dronePos2);
        }

        //manual release for plungers
        if(gamepad2.right_bumper && !gamepad2prev.right_bumper){
            if(plungerLClosed){
                pLeft.setPosition(plungerGrabPos);
                plungerLClosed = false;
            }
            else{
                pLeft.setPosition(plungerReleasePos);
                plungerLClosed = true;
            }
        }

        if(gamepad2.left_bumper && !gamepad2prev.left_bumper){
            if(plungerRClosed){
                pRight.setPosition(plungerGrabPos);
                plungerRClosed = false;
            }
            else{
                pRight.setPosition(plungerReleasePos);
                plungerRClosed = true;
            }
        }

        //output automation
        if(gamepad2.dpad_left && !gamepad2prev.dpad_left){
            telemetry.addData("Abcdef", 0);

            conveyor.setPower(0.0);
            pLeft.setPosition(plungerReleasePos);
            pRight.setPosition(plungerReleasePos);
            slidePositionTarget = 200.0;

            /*DelaysAndAutoms armManeuver = */new DelaysAndAutoms(100.0, armLeft, armOutPos, armInPos);
            /*DelaysAndAutoms slidePositionDelay = */new DelaysAndAutoms(200.0, slidePositionTarget, 200.0, 0.0);
            /*DelaysAndAutoms pLeftDelay = */new DelaysAndAutoms(400.0, pLeft, plungerReleasePos, plungerGrabPos);
            /*DelaysAndAutoms pRightDelay = */new DelaysAndAutoms(400.0, pRight, plungerReleasePos, plungerGrabPos);
        }

        if(gamepad2.left_stick_y < -0.1 && (slidePositionTarget >= 1000) && (armLeft.getPosition() > 0.8) && !zeroing){
            armLeft.setPosition(armOutPos);
        }

        gamepad1prev.copy(gamepad1);
        gamepad2prev.copy(gamepad2);

        /*telemetry.addData("rf", postRF);
        telemetry.addData("lf", postLF);
        telemetry.addData("rb", postRB);
        telemetry.addData("lb", postLB);*/

        telemetry.addData("intake", intake.getPower());
        //telemetry.addData("armRight", "Position: " + armRight.getPosition());
        telemetry.addData("armLeft", "Position: " + armLeft.getPosition());
        telemetry.addData("pRight", "Position: " + pRight.getPosition());
        telemetry.addData("pLeft", "Position: " + pLeft.getPosition());
        telemetry.addData("intakeRight", "Position: " + intakeRight.getPosition());
        telemetry.addData("intakeLeft", "Position: " + intakeLeft.getPosition());
        telemetry.addData("rs", "Power: " + rs.getPower());
        telemetry.addData("ls", "Power: " + ls.getPower());
        telemetry.addData("drone", "Position: " + droneLauncher.getPosition());
        //telemetry.addData("slides target", "target: " + slidePositionTarget);
        telemetry.addData("slides position rs: ", "current rs pos: " + rs.getCurrentPosition());
        telemetry.addData("slides position ls: ", "current ls pos: " + ls.getCurrentPosition());
        telemetry.addData("limit switch", slidesLimit.isPressed());
        telemetry.addData("leftDS", "Distance B from backdrop: " + leftDS.getDistance(DistanceUnit.INCH));
        telemetry.addData("rightDS", "Distance A from backdrop: " + rightDS.getDistance(DistanceUnit.INCH));
        telemetry.addData("allDelays: ", DelaysAndAutoms.allDelays.size());
        if(DelaysAndAutoms.allDelays.size() > 0) {
            telemetry.addData("ALLLEX", DelaysAndAutoms.allDelays.get(0).delayTimer.milliseconds());
        }

        packet.put("slides target", slidePositionTarget);
        packet.put("slides position rs", rs.getCurrentPosition());
        packet.put("slides position ls", ls.getCurrentPosition());
        packet.put("slides power rs", rs.getPower());
        packet.put("slides power ls", ls.getPower());
        packet.put("slides limit pressed", slidesLimit.isPressed());
        packet.put("slide saved position", slideSavedPosition);
        packet.put("scalar", scalar);
        packet.put("target distanceBackdrop", distBackdrop);
        packet.put("leftDistBackdrop", leftDS.getDistance(DistanceUnit.INCH));
        packet.put("rightDistBackdrop", rightDS.getDistance(DistanceUnit.INCH));
        dashboard.sendTelemetryPacket(packet);

        updateDelays();
    }
}
