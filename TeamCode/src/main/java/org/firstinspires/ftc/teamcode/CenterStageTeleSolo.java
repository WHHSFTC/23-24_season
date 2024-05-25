package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
@Config
public class CenterStageTeleSolo extends CenterStageTeleProper {

    @Override
    public void init() {
        super.init();
    }
    @Override
    public void childLoop() {
        slidesPidRight.update(rs.getCurrentPosition(), timeGap);
        slidesPidLeft.update(ls.getCurrentPosition(), timeGap);
        //find timeGap
        timeGap = timer.milliseconds();
        timer.reset();

        double scalar = 1.0;

        double y = -gamepad1.left_stick_x; //verticals
        double x = -gamepad1.left_stick_y * 1.1; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation

        /*if (fieldCentric) {
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
        telemetry.addData("field centric:", fieldCentric);*/

        if (gamepad1.a) {
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            distancePower = (Math.abs(yaw) < Math.toRadians(45.0)) ? DSpid.distancePID(rightDS.getDistance(DistanceUnit.INCH), leftDS.getDistance(DistanceUnit.INCH) - leftDSOffset, timeGap, distBackdrop + 2.0 * gamepad1.left_trigger - 2.0 * gamepad1.right_trigger): 0.0;
            anglePower = DSpid.anglePID(yaw, timeGap, Math.toRadians(0.0));
            x = x > 0.55 ? x : -distancePower;
            r = -anglePower;
            scalar = 0.25;
        }

        //heading lock
        /*if (gamepad1.b) {
            if (!gamepad1prev.b) {
                targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }
            anglePower = DSpid.anglePID(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), timeGap, targetYaw);
            r = -anglePower;
        }*/

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

        telemetry.addData("Slide target: ", slidePositionTarget);
        telemetry.addData("Error RS", "Error RS: " + (slidePositionTarget - rs.getCurrentPosition()));
        telemetry.addData("Error LS", "Error LS: " + (slidePositionTarget - ls.getCurrentPosition()));

        if (!zeroing) {
            rs.setPower(slidesPidRight.calculatePower(slidePositionTarget));
            ls.setPower(slidesPidLeft.calculatePower(slidePositionTarget));
        }

        //TURTLE
        /*if (gamepad1.left_bumper && scalar > 0.3) {
            scalar = 0.3;
        }*/

        double preRF = 1.3 * r * Math.sqrt(scalar) + y * Math.cbrt(scalar) + x * scalar;
        double preLF = 1.3 * r * Math.sqrt(scalar) + y * Math.cbrt(scalar) - x * scalar;
        double preRB = 1.3 * r * Math.sqrt(scalar) - y * Math.cbrt(scalar) + x * scalar;
        double preLB = 1.3 * r * Math.sqrt(scalar) - y * Math.cbrt(scalar) - x * scalar;


        double max = Math.max(Math.max(Math.max(Math.max(preRF, preRB), preLB), preLF), 1);


        rf.setPower(preRF / max);
        lf.setPower(preLF / max);
        rb.setPower(preRB / max);
        lb.setPower(preLB / max);

        //AUTOMATION
        if(gamepad1.y && !gamepad1prev.y && slidePositionTarget < 250.0){
            slidePositionTarget = 0.0;
            new DelaysAndAutoms(300.0, pLeft, plungerReleasePos, plungerGrabPos);
            new DelaysAndAutoms(300.0, pRight, plungerReleasePos, plungerGrabPos);
            plungerRClosed = false;
            plungerLClosed = false;
        }

        if(gamepad1.x && !gamepad1prev.x){
            pLeft.setPosition(plungerReleasePos);
            pRight.setPosition(plungerReleasePos);
            slidePositionTarget = 350.0;
            new DelaysAndAutoms(150.0, armLeft, armOutPos, armInPos);
            slideUpdate = new DelaysAndAutoms(400.0, slidePositionTarget, 117.0);
        }

        if(gamepad1.b && !gamepad1prev.b && armLeft.getPosition() > 0.8 && slidePositionTarget < 250){
            slidePositionTarget = 350.0;
            new DelaysAndAutoms(300.0, armLeft, armInPos, armOutPos);
            slideUpdate = new DelaysAndAutoms(400.0, slidePositionTarget, 0.0);
        }

        if(slideSavedPosition == 0){
            slideSavedPosition = 500.0;
        }

        if(!gamepad1.y && (armLeft.getPosition() > 0.8) && (pRight.getPosition() < 0.2) && (pLeft.getPosition() < 0.2) && (slidePositionTarget < 5) && slideDelay == null){
            new DelaysAndAutoms(500.0, armLeft, armInPos, armOutPos);
            slideDelay = new DelaysAndAutoms(200.0, slidePositionTarget, slideSavedPosition);
        }

        if(gamepad1.y && !gamepad1prev.y && slidePositionTarget > 450){
            new DelaysAndAutoms(100.0, armLeft, armOutPos, armInPos);
            new DelaysAndAutoms(300.0, armLeft, armInPos, armOutPos);
        }

        //plunger close
        if (gamepad1.b && slidePositionTarget > 400 && armLeft.getPosition() < 0.2) {
            pRight.setPosition(plungerReleasePos);
            pLeft.setPosition(plungerReleasePos);
            plungerRClosed = true;
            plungerLClosed = true;
        }

        /*if (gamepad1.dpad_right && !zeroing) {
            slidePositionTarget = slideSavedPosition;
        }*/

        if (gamepad1.dpad_down && !zeroing && (pLeft.getPosition() > 0.9 && pRight.getPosition() > 0.9)) {
            if (!gamepad2prev.dpad_down) {
                slideSavedPosition = slidePositionTarget;
                slidePositionTarget = slideMin;
                zeroing = true;
            }
        }

        //slides increments
        if (gamepad1.dpad_down && !zeroing && (pLeft.getPosition() < 0.2 || pRight.getPosition() < 0.2)) {
            if (!gamepad2prev.dpad_down) {
                slidePositionTarget -= 315.0;
                slideIncrement--;
                if (slidePositionTarget < 550) {
                    slidePositionTarget = 550;
                }
            }
        }

        if (gamepad1.dpad_up) {
            if (!gamepad2prev.dpad_up) {
                if (slidePositionTarget > 2800) {
                    slidePositionTarget = 2800;
                }
                slidePositionTarget += 315.0;
                slideIncrement++;
            }
        }

        //intake spinning
        if (gamepad1.left_trigger > 0.2) {
            intake.setPower(-0.3 * gamepad1.left_trigger); //reverse
            conveyor.setPower(0.9);
        } else if (gamepad1.right_trigger > 0.2) {
            intake.setPower(gamepad1.right_trigger * 0.5); //forward
            conveyor.setPower(-0.9);
        } else {
            intake.setPower(0.0);
            conveyor.setPower(0.0);
        }

        //intake swinging out and swinging in
        /*if (gamepad1.right_bumper && !gamepad1prev.right_bumper) {
            if (intakeOnGround) {
                intakeRight.setPosition(intakeUpPos);
                intakeLeft.setPosition(intakeUpPos);
                intakeOnGround = false;
            } else {
                intakeRight.setPosition(intakeDownPos);
                intakeLeft.setPosition(intakeDownPos);
                intakeOnGround = true;
            }
        }*/

        //stack position intake
        if (gamepad1.dpad_right) {
            if (!intakeOnGround) {
                intakeOnGround = true;
            }
            intakeRight.setPosition(intakeStackPos);
            intakeLeft.setPosition(intakeStackPos);
        }

        //reset imu
        if (gamepad1.start) {
            imu.resetYaw();
            zeroing = true;
        }

        //manual reset for slides
        /*if (gamepad1.start) {
            zeroing = true;
        }*/

        if (zeroing) {
            slidePositionTarget = 0.0;
            rs.setPower(-0.6);
            ls.setPower(-0.6);
        }
        if (slidesLimit.isPressed()) {
            zeroing = false;
        }

        // drone launcher
        if (gamepad1.back) {
            droneLauncher.setPosition(dronePos1);
        } else {
            droneLauncher.setPosition(dronePos2);
        }

        //manual release for plungers
        if (gamepad1.right_bumper && !gamepad1prev.right_bumper) {
            if (plungerLClosed) {
                pLeft.setPosition(plungerGrabPos);
                plungerLClosed = false;
            } else {
                pLeft.setPosition(plungerReleasePos);
                plungerLClosed = true;
            }
        }

        if (gamepad1.left_bumper && !gamepad1prev.left_bumper) {
            if (plungerRClosed) {
                pRight.setPosition(plungerGrabPos);
                plungerRClosed = false;
            } else {
                pRight.setPosition(plungerReleasePos);
                plungerRClosed = true;
            }
        }

        //hang slides position
        if (gamepad1.dpad_left && !gamepad1prev.dpad_left) {
            if (!gamepad1prev.dpad_left) {
                slidePositionTarget = 1750.0;
            }
        }

        gamepad1prev.copy(gamepad1);
        gamepad2prev.copy(gamepad2);

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
        telemetry.addData("slides position rs: ", "current rs pos: " + rs.getCurrentPosition());
        telemetry.addData("slides position ls: ", "current ls pos: " + ls.getCurrentPosition());
        telemetry.addData("limit switch", slidesLimit.isPressed());
        telemetry.addData("allDelays: ", DelaysAndAutoms.allDelays.size());
        telemetry.addData("slide increment: ", slideIncrement);
        telemetry.addData("slide saved position", slideSavedPosition);

        packet.put("slides target", slidePositionTarget);
        packet.put("slides position rs", rs.getCurrentPosition());
        packet.put("slides position ls", ls.getCurrentPosition());
        packet.put("slides power rs", rs.getPower());
        packet.put("slides power ls", ls.getPower());
        packet.put("slides limit pressed", slidesLimit.isPressed());
        packet.put("slide saved position", slideSavedPosition);
        packet.put("scalar", scalar);
        packet.put("target distanceBackdrop", distBackdrop);
        dashboard.sendTelemetryPacket(packet);
    }
}
