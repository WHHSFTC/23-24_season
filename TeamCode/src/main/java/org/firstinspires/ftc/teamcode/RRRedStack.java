
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
public class RRRedStack extends CenterStageAuto{

    Trajectory purplePixel1;
    Trajectory purplePixel2;
    Trajectory purplePixel3;
    TrajectorySequence moveUp1;
    TrajectorySequence moveUp2;
    TrajectorySequence moveUp3;
    Trajectory yellowPixel1;
    Trajectory yellowPixel2;
    Trajectory yellowPixel3;
    Trajectory reset1;
    Trajectory reset2;
    Trajectory reset3;
    Trajectory park1;
    Trajectory park2;
    Trajectory park3;

    @Override
    public void init(){
        blue = false;
        isRightSideHardForCameraToSee = true;
        super.init();
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-37.36, -63.28,Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        purplePixel1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-45.3, -34.5),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    armLeft.setPosition(armInPos);
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .build();

        purplePixel2 = drive.trajectoryBuilder(startPose, true)
                .lineTo(new Vector2d(-33.6,-32.0),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    armLeft.setPosition(armInPos);
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .build();

        purplePixel3 = drive.trajectoryBuilder(startPose, true)
                .lineToLinearHeading(new Pose2d(-28.4,-30.9, Math.toRadians(210)),
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
        moveUp1 = drive.trajectorySequenceBuilder(purplePixel1.end())
                .forward(8,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .strafeLeft(11.0,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .back(25.0,SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(-28.9, -10.3, Math.toRadians(181)), 0,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(60.0,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        moveUp2 = drive.trajectorySequenceBuilder(purplePixel2.end())
                .forward(8, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .strafeRight(17.0, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .back(20.0, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(-28.9, -10.3, Math.toRadians(180)), 0,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(60.0,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        moveUp3 = drive.trajectorySequenceBuilder(purplePixel3.end())
                .forward(15, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(-28.9, -10.3, Math.toRadians(180)), 0,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(60.0,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //yellow pixels
        yellowPixel1 = drive.trajectoryBuilder(moveUp1.end())
                .lineToLinearHeading(new Pose2d(53.3,-30.7, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.2,0,()->{
                    slidePositionTarget = 700.0;
                })
                .addTemporalMarker(0.5,0,()->{
                    armLeft.setPosition(0.05);
                })
                .addDisplacementMarker(()->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })//
                .build();

        yellowPixel2 = drive.trajectoryBuilder(moveUp2.end())
                .lineToLinearHeading(new Pose2d(52.8,-36.5, Math.toRadians(180.0)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.2,0,()->{
                    slidePositionTarget = 700.0;
                })
                .addDisplacementMarker(0.5,0,()->{
                    armLeft.setPosition(0.05);
                })
                .addDisplacementMarker(()->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .build();

        yellowPixel3 = drive.trajectoryBuilder(moveUp3.end())
                .lineToSplineHeading(new Pose2d(53.3,-38.0, Math.toRadians(181)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.2,0,()->{
                    slidePositionTarget = 700.0;
                })
                .addDisplacementMarker(0.5,0,()->{
                    armLeft.setPosition(0.05);
                })
                .addDisplacementMarker(()->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .build();

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

        //parks
        park1 = drive.trajectoryBuilder(reset1.end())
                .splineToLinearHeading(new Pose2d(56.7,-12.0, Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        park2 = drive.trajectoryBuilder(reset2.end())
                .splineToLinearHeading(new Pose2d(56.7,-12.0, Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        park3 = drive.trajectoryBuilder(reset3.end())
                .lineToLinearHeading(new Pose2d(56.7,-50.0, Math.toRadians(182)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
    }
    @Override
    public void childLoop() {
        super.childLoop();
        telemetry.addData("target", slidePositionTarget);
        telemetry.addData("loopvision", pipeline.getOutput());
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
        super.followPurple();
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
        super.followMOVEUP();
    }

    @Override
    public void followYellow(){
        if(elementPosition == 0){
            drive.followTrajectoryAsync(yellowPixel1);
        }
        else if(elementPosition == 1){
            drive.followTrajectoryAsync(yellowPixel2);
        }
        else{
            drive.followTrajectoryAsync(yellowPixel3);
        }
        liftTimer.reset();
        super.followYellow();
    }
    @Override
    public void followReset(){
        if(elementPosition == 0){
            drive.followTrajectoryAsync(reset1);
        }
        else if(elementPosition == 1){
            drive.followTrajectoryAsync(reset2);
        }
        else{
            drive.followTrajectoryAsync(reset3);
        }
        liftTimer.reset();

        if (!drive.isBusy()) {
            currentState = AutoState.PARK;
            followPark();
        }
    }

    @Override
    public void followToStack(){

    }

    @Override
    public void followPark(){
        slidePositionTarget = 0.0;
        intakeLeft.setPosition(intakeDownPos);
        intakeRight.setPosition(intakeDownPos);
        if(elementPosition == 0){
            drive.followTrajectoryAsync(park1);
        }
        else if(elementPosition == 1){
            drive.followTrajectoryAsync(park2);
        }
        else{
            drive.followTrajectoryAsync(park3);
        }
        super.followPark();
    }
}