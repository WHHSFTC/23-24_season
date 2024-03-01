
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
public class StackAutoRedBackdrop extends RRBlueBackdrop {

    Trajectory purplePixel1;
    Trajectory purplePixel2;
    Trajectory purplePixel3;
    Trajectory moveUp1;
    Trajectory moveUp2;
    Trajectory moveUp3;
    Trajectory yellowPixel1;
    Trajectory yellowPixel2;
    Trajectory yellowPixel3;
    Trajectory reset1;
    Trajectory reset2;
    Trajectory reset3;
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

        blue = true;
        isRightSideHardForCameraToSee = false;
        super.init();
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14.96, -63.25, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        purplePixel1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(3.8, -36, Math.toRadians(300)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    armLeft.setPosition(armInPos);
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .build();

        purplePixel2 = drive.trajectoryBuilder(startPose, true)
                .lineTo(new Vector2d(9,-32.5),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    armLeft.setPosition(armInPos);
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .build();

        purplePixel3 = drive.trajectoryBuilder(startPose, true)
                .lineTo(new Vector2d(23.5,-35),
                        SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    armLeft.setPosition(armInPos);
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .build();

        //move up
        moveUp1 = drive.trajectoryBuilder(purplePixel1.end())
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        moveUp2 = drive.trajectoryBuilder(purplePixel2.end())
                .forward(5, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        moveUp3 = drive.trajectoryBuilder(purplePixel3.end())
                .forward(5, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        //yellow pixels
        yellowPixel1 = drive.trajectoryBuilder(moveUp1.end())
                .lineToLinearHeading(new Pose2d(51.3,-34.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.2,0,()->{
                    slidePositionTarget = 700.0;
                })
                .addTemporalMarker(0.5,0,()->{
                    armLeft.setPosition(armOutPos);
                })
                .addDisplacementMarker(()->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })//
                .build();

        yellowPixel2 = drive.trajectoryBuilder(moveUp2.end())
                .lineToLinearHeading(new Pose2d(51.3,-41, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.2,0,()->{
                    slidePositionTarget = 700.0;
                })
                .addDisplacementMarker(0.5,0,()->{
                    armLeft.setPosition(armOutPos);
                })
                .addDisplacementMarker(()->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .build();

        yellowPixel3 = drive.trajectoryBuilder(moveUp3.end())
                .lineToSplineHeading(new Pose2d(51.3,-48.0, Math.toRadians(181)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.2,0,()->{
                    slidePositionTarget = 700.0;
                })
                .addDisplacementMarker(0.5,0,()->{
                    armLeft.setPosition(armOutPos);
                })
                .addDisplacementMarker(()->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .build();

        //resets
        reset1 = drive.trajectoryBuilder(yellowPixel1.end())
                .forward(8,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0.8,0,()->{
                    slidePositionTarget = 0.0;
                })
                .build();

        reset2 = drive.trajectoryBuilder(yellowPixel2.end())
                .forward(8,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0.8,0,()->{
                    slidePositionTarget = 0.0;
                })
                .build();

        reset3 = drive.trajectoryBuilder(yellowPixel3.end())
                .forward(10,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0.8,0,()->{
                    slidePositionTarget = 0.0;
                })
                .build();

        driveToStack1 = drive.trajectorySequenceBuilder(reset1.end())
                .lineToLinearHeading(new Pose2d(43.65, -10.55, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intakeLeft.setPosition(intakeStackPos);
                    intakeRight.setPosition(intakeStackPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.setPower(0.7);
                    conveyor.setPower(-0.9);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.45, ()->{
                    slidePositionTarget = 350.0;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
                    armLeft.setPosition(armInPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{
                    slidePositionTarget = 150.0;
                })
                .lineToLinearHeading(new Pose2d(-42.38, -17.2, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(-55.68,-17.15, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        driveToStack2 = drive.trajectorySequenceBuilder(reset2.end())
                .lineToLinearHeading(new Pose2d(43.65, -10.55, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intakeLeft.setPosition(intakeStackPos);
                    intakeRight.setPosition(intakeStackPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.setPower(0.7);
                    conveyor.setPower(-0.9);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.45, ()->{
                    slidePositionTarget = 350.0;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
                    armLeft.setPosition(armInPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{
                    slidePositionTarget = 150.0;
                })
                .lineToLinearHeading(new Pose2d(-42.38, -17.2, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(-55.68,-17.15, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        driveToStack3 = drive.trajectorySequenceBuilder(reset3.end())
                .lineToLinearHeading(new Pose2d(43.65, -10.55, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                ).UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intakeLeft.setPosition(intakeStackPos);
                    intakeRight.setPosition(intakeStackPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.setPower(0.7);
                    conveyor.setPower(-0.9);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.45, ()->{
                    slidePositionTarget = 350.0;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
                    armLeft.setPosition(armInPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{
                    slidePositionTarget = 150.0;
                })
                .lineToLinearHeading(new Pose2d(-42.38, -17.2, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(new Pose2d(-55.68,-17.15, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                .back(7, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(5, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                .back(7, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(5, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                .back(7, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(5, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                .lineToLinearHeading(new Pose2d(-30, 10, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(2.0, () -> {
                    slidePositionTarget = 700.0;
                })
                .UNSTABLE_addTemporalMarkerOffset(2.3, () -> {
                    armLeft.setPosition(armOutPos);
                    conveyor.setPower(0.0);
                })
                .lineToLinearHeading(new Pose2d(37.65, -10.55, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(50.5, -45, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->{
                    pRight.setPosition(plungerReleasePos);
                    pLeft.setPosition(plungerReleasePos);
                })
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
                .lineToLinearHeading(new Pose2d(-30, 10, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(2.0, () -> {
                    slidePositionTarget = 700.0;
                })
                .UNSTABLE_addTemporalMarkerOffset(2.3, () -> {
                    armLeft.setPosition(armOutPos);
                    conveyor.setPower(0.0);
                })
                .lineToLinearHeading(new Pose2d(37.65, -10.55, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(50.5, -38, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->{
                    pRight.setPosition(plungerReleasePos);
                    pLeft.setPosition(plungerReleasePos);
                })
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
                .lineToLinearHeading(new Pose2d(-30, 10, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(2.0, () -> {
                    slidePositionTarget = 700.0;
                })
                .UNSTABLE_addTemporalMarkerOffset(2.3, () -> {
                    armLeft.setPosition(armOutPos);
                    conveyor.setPower(0.0);
                })
                .lineToLinearHeading(new Pose2d(37.65, -10.55, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(50.5, -30.3, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->{
                    pRight.setPosition(plungerReleasePos);
                    pLeft.setPosition(plungerReleasePos);
                })
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
                .splineToLinearHeading(new Pose2d(55.7, -11, Math.toRadians(180)), Math.toRadians(0),
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
                .splineToLinearHeading(new Pose2d(55.7, -11, Math.toRadians(180)), Math.toRadians(0),
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
                .lineToLinearHeading(new Pose2d(55.7, -50.0, Math.toRadians(180)),
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
                    currentState = AutoState.YELLOW;
                    followYellow();
                }
                break;
            case YELLOW:
                if (!drive.isBusy()) {
                    currentState = AutoState.RESET;
                    followReset();
                }
                break;
            case RESET:
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
                    currentState = AutoState.OUTPUT;
                    followOutput();
                }
                break;
            case OUTPUT:
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
    public void followOutput(){
        super.followOutput();
        if(rightDS.getDistance(DistanceUnit.INCH) < 6.5 && leftDS.getDistance(DistanceUnit.INCH) < 6.5){
            pLeft.setPosition(plungerReleasePos);
            pRight.setPosition(plungerReleasePos);
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