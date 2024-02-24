package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class CenterStageTeleSoloBlue extends CenterStageTeleProper {
    @Override
    public void childLoop() {
        slidesPidRight.update(rs.getCurrentPosition(), timeGap);
        slidesPidLeft.update(ls.getCurrentPosition(), timeGap);
        //find timeGap
        timeGap = timer.milliseconds();
        timer.reset();

        double scalar = 1.0;

        double y = -gamepad1.left_stick_x; //verticals
        double x; //horizontal
        if (!outputMode) {
            x = -gamepad1.left_stick_y * 1.1;
        } else {
            x = 0.0;
        }
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

        if (gamepad1.left_trigger > 0.5) {
            distancePower = DSpid.distancePID(rightDS.getDistance(DistanceUnit.INCH), leftDS.getDistance(DistanceUnit.INCH) - leftDSOffset, timeGap, distBackdrop);
            anglePower = DSpid.anglePID(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), timeGap, Math.toRadians(0));
            x = x > 0.55 ? x : -distancePower;
            r = -anglePower;
            scalar = 0.25;
            outputMode = true;
        } else {
            outputMode = false;
        }

        //heading lock
        if (gamepad1.b) {
            if (!gamepad1prev.b) {
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

        if (outputMode) {
            if (Math.abs(gamepad1.left_stick_y) > 0.01) {
                zeroing = false;

                if (Math.abs(gamepad1.left_stick_y) > 0.01) {
                    slidePositionTarget -= slideTargetGain * gamepad1.left_stick_y;
                }

                if (slidePositionTarget < slideMin) {
                    slidePositionTarget = slideMin;
                }
                if (slidePositionTarget > slideMax) {
                    slidePositionTarget = slideMax;
                }
                if (slidePositionTarget > 600.0) {
                    scalar = 1.0 - (Math.sqrt(slidePositionTarget / slideMax) / 1.25);
                }
            }
        }

        telemetry.addData("Slide target: ", slidePositionTarget);
        telemetry.addData("Error RS", "Error RS: " + (slidePositionTarget - rs.getCurrentPosition()));
        telemetry.addData("Error LS", "Error LS: " + (slidePositionTarget - ls.getCurrentPosition()));

        if (!zeroing) {
            rs.setPower(slidesPidRight.calculatePower(slidePositionTarget));
            ls.setPower(slidesPidLeft.calculatePower(slidePositionTarget));
        }

        if (gamepad1.left_bumper && scalar > 0.3) {
            scalar = 0.3;
        }

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
        if(gamepad1.y && !gamepad1prev.y){
            pLeft.setPosition(plungerReleasePos);
            pRight.setPosition(plungerReleasePos);
            slidePositionTarget = 350.0;
            new DelaysAndAutoms(150.0, armLeft, armOutPos, armInPos);
            slideUpdate = new DelaysAndAutoms(400.0, slidePositionTarget, 0.0);
            new DelaysAndAutoms(700.0, pLeft, plungerReleasePos, plungerGrabPos);
            new DelaysAndAutoms(700.0, pRight, plungerReleasePos, plungerGrabPos);
            plungerRClosed = false;
            plungerLClosed = false;
        }

        if(!gamepad1.y && (armLeft.getPosition() > 0.8) && (pRight.getPosition() < 0.2) && (pLeft.getPosition() < 0.2) && (slidePositionTarget < 5)){
            slideUpdate = new DelaysAndAutoms(50.0, slidePositionTarget, slideSavedPosition);
            new DelaysAndAutoms(300.0, armLeft, armInPos, armOutPos);
        }

        if (outputMode) {
            //arm swings out
            if (gamepad1.x) {
                //armRight.setPosition(0.0);
                armLeft.setPosition(armOutPos);
            }

            //arm swings in
            if (gamepad1.b) {
                //armRight.setPosition(1.0);
                armLeft.setPosition(armInPos);
            }

            //plunger close
            if (gamepad1.x) {
                pRight.setPosition(plungerReleasePos);
                pLeft.setPosition(plungerReleasePos);
                plungerRClosed = true;
                plungerLClosed = true;
            }

            if (gamepad1.dpad_right && !zeroing) {
                slidePositionTarget = slideSavedPosition;
            }
        }

        if (gamepad1.dpad_down && !zeroing && (pLeft.getPosition() > 0.9 && pRight.getPosition() > 0.9)) {
            if (!gamepad2prev.dpad_down) {
                slideSavedPosition = slidePositionTarget;
                slidePositionTarget = slideMin;
                slideIncrement = 0;
                zeroing = true;
            }
        }

        //slides increments
        if (gamepad1.dpad_down && !zeroing && pLeft.getPosition() < 0.2 && pRight.getPosition() < 0.2) {
            if (!gamepad2prev.dpad_down) {
                slidePositionTarget -= 400.0;
                slideIncrement--;
                if (slidePositionTarget < 600) {
                    slidePositionTarget = 600;
                }
            }
        }

        if (gamepad1.dpad_up) {
            if (!gamepad2prev.dpad_up) {
                if (slidePositionTarget > 2800) {
                    slidePositionTarget = 2800;
                }
                if (slideIncrement == 0) {
                    slidePositionTarget += 550.0;
                } else {
                    slidePositionTarget += 350.0;
                }
                slideIncrement++;
            }
        }

        //intake spinning
        if (!outputMode) {
            if (gamepad1.right_bumper) {
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
            if (gamepad1.right_bumper || gamepad1.right_trigger > 0.2) {
                if (gamepad1.right_trigger > 0.9) {
                    intakeRight.setPosition(intakeDownPos);
                    intakeLeft.setPosition(intakeDownPos);
                } else {
                    intakeRight.setPosition(intakeStackPos);
                    intakeLeft.setPosition(intakeStackPos);
                }
            } else {
                intakeRight.setPosition(intakeUpPos);
                intakeLeft.setPosition(intakeUpPos);
            }

            //stack position intake
            if (gamepad1.dpad_right) {
                if (!intakeOnGround) {
                    intakeOnGround = true;
                }
                intakeRight.setPosition(intakeStackPos);
                intakeLeft.setPosition(intakeStackPos);
            }
        }

        //reset imu
        if (gamepad1.start || gamepad2.start) {
            imu.resetYaw();
        }

        //manual reset for slides
        if (gamepad1.start) {
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

        // drone launcher
        if (gamepad1.back) {
            droneLauncher.setPosition(dronePos1);
        } else {
            droneLauncher.setPosition(dronePos2);
        }


        if (outputMode) {
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
        }

        //hang slides position
        if (gamepad1.dpad_left) {
                slidePositionTarget = 1800.0;
        }

        if (gamepad1.left_stick_y < -0.1 && (slidePositionTarget >= 400) && (armLeft.getPosition() > 0.8) && !zeroing) {
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
        //telemetry.addData("leftDS", "Distance B from backdrop: " + leftDS.getDistance(DistanceUnit.INCH));
        //telemetry.addData("rightDS", "Distance A from backdrop: " + rightDS.getDistance(DistanceUnit.INCH));
        telemetry.addData("allDelays: ", DelaysAndAutoms.allDelays.size());
        telemetry.addData("slide increment: ", slideIncrement);
        telemetry.addData("automationDone: ", automationDone);
        if (DelaysAndAutoms.allDelays.size() > 0) {
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
        //packet.put("leftDistBackdrop", leftDS.getDistance(DistanceUnit.INCH));
        //packet.put("rightDistBackdrop", rightDS.getDistance(DistanceUnit.INCH));
        dashboard.sendTelemetryPacket(packet);
    }
}