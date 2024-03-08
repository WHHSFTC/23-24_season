
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.opencv.core.Mat;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous (preselectTeleOp = "CenterStageTeleProper")
public class StackAutoRedStack extends RRRedStack {

    Trajectory purplePixel1;
    Trajectory purplePixel2;
    Trajectory purplePixel3;
    TrajectorySequence moveUp1;
    TrajectorySequence moveUp2;
    TrajectorySequence moveUp3;
    TrajectorySequence park1;
    TrajectorySequence park2;
    TrajectorySequence park3;
    TrajectorySequence driveToStack1;
    TrajectorySequence driveToStack2;
    TrajectorySequence driveToStack3;
    TrajectorySequence intake1;
    TrajectorySequence intake2;
    TrajectorySequence intake3;
    TrajectorySequence driveFromStack1;
    TrajectorySequence driveFromStack2;
    TrajectorySequence driveFromStack3;

    @Override
    public void init() {

        blue = false;
        isRightSideHardForCameraToSee = true;
        super.init();
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-37.36, -63.28,Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        purplePixel1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-46.0, -34.5),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    armLeft.setPosition(armInPos);
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .build();

        purplePixel2 = drive.trajectoryBuilder(startPose, true)
                .lineTo(new Vector2d(-33.6,-32.0),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    armLeft.setPosition(armInPos);
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .build();

        purplePixel3 = drive.trajectoryBuilder(startPose, true)
                .lineToLinearHeading(new Pose2d(-28.4,-30.9, Math.toRadians(210)),
                        SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    armLeft.setPosition(armInPos);
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .build();

        //move up
        moveUp1 = drive.trajectorySequenceBuilder(purplePixel1.end())
                .forward(8,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(-30.9, -42.5, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-30.9, -12.2, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-45.00, -12.44, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        moveUp2 = drive.trajectorySequenceBuilder(purplePixel2.end())
                .forward(8, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(-46.0, -40.0, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-46.0, -18.40, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-45.25, -18.44, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        moveUp3 = drive.trajectorySequenceBuilder(purplePixel3.end())
                .forward(15, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(-45.00, -16.44, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        driveToStack1 = drive.trajectorySequenceBuilder(moveUp1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                    conveyor.setPower(-0.9);
                    intake.setPower(0.9);
                    intakeLeft.setPosition(0.51);
                    intakeRight.setPosition(0.51);
                    slidePositionTarget = 150.0;
                })
                .lineToLinearHeading(new Pose2d(-54.6,-13.00, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        driveToStack2 = drive.trajectorySequenceBuilder(moveUp2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                    conveyor.setPower(-0.9);
                    intake.setPower(0.9);
                    intakeLeft.setPosition(0.51);
                    intakeRight.setPosition(0.51);
                    slidePositionTarget = 150.0;
                })
                .lineToLinearHeading(new Pose2d(-54.6,-13.00, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        driveToStack3 = drive.trajectorySequenceBuilder(moveUp3.end())
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                    conveyor.setPower(-0.9);
                    intake.setPower(0.9);
                    intakeLeft.setPosition(0.535);
                    intakeRight.setPosition(0.542);
                    slidePositionTarget = 150.0;
                })
                .lineToLinearHeading(new Pose2d(-54.6,-13.00, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        intake1 = drive.trajectorySequenceBuilder(driveToStack1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.01,()->{
                    intakeLeft.setPosition(intakeUpPos);
                    intakeRight.setPosition(intakeUpPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intakeLeft.setPosition(intakeDownPos);
                    intakeRight.setPosition(intakeDownPos);
                })
                .back(9, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(4.4, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        intake2 = drive.trajectorySequenceBuilder(driveToStack2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.01,()->{
                    intakeLeft.setPosition(intakeUpPos);
                    intakeRight.setPosition(intakeUpPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intakeLeft.setPosition(intakeDownPos);
                    intakeRight.setPosition(intakeDownPos);
                })
                .back(9, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(4.4, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        intake3 = drive.trajectorySequenceBuilder(driveToStack3.end())
                .UNSTABLE_addTemporalMarkerOffset(0.01,()->{
                    intakeLeft.setPosition(intakeUpPos);
                    intakeRight.setPosition(intakeUpPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intakeLeft.setPosition(intakeDownPos);
                    intakeRight.setPosition(intakeDownPos);
                })
                .back(9, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(4.4, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //drive from stack
        driveFromStack1 = drive.trajectorySequenceBuilder(intake1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    conveyor.setPower(-0.9);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    intakeRight.setPosition(intakeUpPos);
                    intakeLeft.setPosition(intakeUpPos);
                    intake.setPower(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.1, () -> {
                    slidePositionTarget = 0.0;
                })
                .UNSTABLE_addTemporalMarkerOffset(2.4, () -> {
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .lineToLinearHeading(new Pose2d(-30, -3, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(2.0, () -> {
                    slidePositionTarget = 900.0;
                })
                .UNSTABLE_addTemporalMarkerOffset(2.3, () -> {
                    armLeft.setPosition(armOutPos);
                    conveyor.setPower(0.0);
                })
                .lineToLinearHeading(new Pose2d(37.65, -10.55, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(1.8, ()->{
                    pRight.setPosition(plungerReleasePos);
                    pLeft.setPosition(plungerReleasePos);
                })
                .lineToLinearHeading(new Pose2d(55.2, -26.0, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        driveFromStack2 = drive.trajectorySequenceBuilder(intake2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    conveyor.setPower(-0.9);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    intakeRight.setPosition(intakeUpPos);
                    intakeLeft.setPosition(intakeUpPos);
                    intake.setPower(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.1, () -> {
                    slidePositionTarget = 0.0;
                })
                .UNSTABLE_addTemporalMarkerOffset(2.4, () -> {
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .lineToLinearHeading(new Pose2d(-30, -3, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(2.0, () -> {
                    slidePositionTarget = 900.0;
                })
                .UNSTABLE_addTemporalMarkerOffset(2.3, () -> {
                    armLeft.setPosition(armOutPos);
                    conveyor.setPower(0.0);
                })
                .lineToLinearHeading(new Pose2d(37.65, -10.55, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(2.2, ()->{
                    pRight.setPosition(plungerReleasePos);
                    pLeft.setPosition(plungerReleasePos);
                })
                .lineToLinearHeading(new Pose2d(55.2, -33.0, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        driveFromStack3 = drive.trajectorySequenceBuilder(intake3.end())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    conveyor.setPower(-0.9);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    intakeRight.setPosition(intakeUpPos);
                    intakeLeft.setPosition(intakeUpPos);
                    intake.setPower(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.1, () -> {
                    slidePositionTarget = 0.0;
                })
                .UNSTABLE_addTemporalMarkerOffset(2.4, () -> {
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .lineToLinearHeading(new Pose2d(-30, -3, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(2.0, () -> {
                    slidePositionTarget = 900.0;
                })
                .UNSTABLE_addTemporalMarkerOffset(2.3, () -> {
                    armLeft.setPosition(armOutPos);
                    conveyor.setPower(0.0);
                })
                .lineToLinearHeading(new Pose2d(37.65, -10.55, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(2.2, ()->{
                    pRight.setPosition(plungerReleasePos);
                    pLeft.setPosition(plungerReleasePos);
                })
                .lineToLinearHeading(new Pose2d(55.2, -40.0, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //parks
        park1 = drive.trajectorySequenceBuilder(driveFromStack1.end())
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    slidePositionTarget = 0.0;
                })
                .splineToLinearHeading(new Pose2d(55.7, -12, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        park2 = drive.trajectorySequenceBuilder(driveFromStack2.end())
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    slidePositionTarget = 0.0;
                })
                .splineToLinearHeading(new Pose2d(55.7, -10, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        park3 = drive.trajectorySequenceBuilder(driveFromStack3.end())
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    slidePositionTarget = 0.0;
                })
                .lineToLinearHeading(new Pose2d(55.7, -15, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
    }
    @Override
    public void childLoop() {
        slidesPidLeft.update(ls.getCurrentPosition(), timePerLoop);
        slidesPidRight.update(rs.getCurrentPosition(), timePerLoop);
        rs.setPower(slidesPidRight.calculatePower(slidePositionTarget));
        ls.setPower(slidesPidLeft.calculatePower(slidePositionTarget));
        if (drive.isBusy()) {
            drive.update();
        }
        telemetry.addData("State: ", currentState);
        telemetry.addData("slides target: ", slidePositionTarget);
        telemetry.addData("robot busy", drive.isBusy());
        switch (currentState) {
            case PURPLE:
                if (!drive.isBusy()) {
                    currentState = AutoState.MOVEUP;
                    followMOVEUP();
                }
                break;
            case MOVEUP:
                if (!drive.isBusy()) {
                    currentState = AutoState.TO_STACK;
                    followToStack();
                }
                break;
            case TO_STACK:
                if (!drive.isBusy()) {
                    currentState = AutoState.INTAKE;
                    followIntake();
                }
                break;
            case INTAKE:
                if (!drive.isBusy()) {
                    currentState = AutoState.FROM_STACK;
                    followFromStack();
                }
                break;
            case FROM_STACK:
                if (!drive.isBusy()) {
                    currentState = AutoState.PARK;
                    followPark();
                }
                break;
            case PARK:
                if (!drive.isBusy()) {
                    currentState = AutoState.IDLE;
                }
                break;
            case IDLE:
                super.stop();
        }
    }

    @Override
    public void followPurple(){
        if(elementPosition == 0){
            drive.followTrajectoryAsync(purplePixel1);
        }
        else if(elementPosition == 1){
            drive.followTrajectoryAsync(purplePixel2);
        }
        else{
            drive.followTrajectoryAsync(purplePixel3);
        }
        liftTimer.reset();
    }

    @Override
    public void followMOVEUP() {
        if(elementPosition == 0){
            drive.followTrajectorySequenceAsync(moveUp1);
        }
        else if(elementPosition == 1){
            drive.followTrajectorySequenceAsync(moveUp2);
        }
        else{
            drive.followTrajectorySequenceAsync(moveUp3);
        }
        liftTimer.reset();
    }
    @Override
    public void followToStack(){
        if(elementPosition == 0){
            drive.followTrajectorySequenceAsync(driveToStack1);
        }
        else if(elementPosition == 1){
            drive.followTrajectorySequenceAsync(driveToStack2);
        }
        else{
            drive.followTrajectorySequenceAsync(driveToStack3);
        }
    }

    @Override
    public void followIntake(){
        if(elementPosition == 0){
            drive.followTrajectorySequenceAsync(intake1);
        }
        else if(elementPosition == 1){
            drive.followTrajectorySequenceAsync(intake2);
        }
        else{
            drive.followTrajectorySequenceAsync(intake3);
        }
    }

    @Override
    public void followFromStack(){
        if(elementPosition == 0){
            drive.followTrajectorySequenceAsync(driveFromStack1);
        }
        else if(elementPosition == 1){
            drive.followTrajectorySequenceAsync(driveFromStack2);
        }
        else{
            drive.followTrajectorySequenceAsync(driveFromStack3);
        }
    }


    @Override
    public void followPark(){
        intakeLeft.setPosition(intakeDownPos);
        intakeRight.setPosition(intakeDownPos);
        if(elementPosition == 0){
            drive.followTrajectorySequence(park1);
        }
        else if(elementPosition == 1){
            drive.followTrajectorySequence(park2);
        }
        else{
            drive.followTrajectorySequence(park3);
        }
    }
}