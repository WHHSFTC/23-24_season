
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
public class RRRedBackdrop extends RRBlueBackdrop{

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
    Trajectory park1;
    Trajectory park2;
    Trajectory park3;

    @Override
    public void init(){
        blue = false;
        super.init();
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14.96, -63.25,Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        /*redBackdrop1 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->{
                    armLeft.setPosition(armInPos);
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(3.2, -35, Math.toRadians(300)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .setReversed(false)
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    slidePositionTarget= 700;
                })
                .UNSTABLE_addTemporalMarkerOffset(3,()->{
                    armLeft.setPosition(armOutPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(4,()->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .lineToSplineHeading(new Pose2d(50.8,-47.2, Math.toRadians(181)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    armLeft.setPosition(armInPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5,()->{
                    slidePositionTarget = 0.0;
                })
                .forward(8,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(58.7,-11, Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        redBackdrop2 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->{
                    armLeft.setPosition(armInPos);
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .setReversed(true)
                .lineTo(new Vector2d(9,-32.5),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .setReversed(false)
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    slidePositionTarget= 700;
                })
                .UNSTABLE_addTemporalMarkerOffset(3,()->{
                    armLeft.setPosition(armOutPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(4,()->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .lineToLinearHeading(new Pose2d(50.8,-41, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    armLeft.setPosition(armInPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5,()->{
                    slidePositionTarget = 0.0;
                })
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(58.7,-11, Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        redBackdrop3 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->{
                    armLeft.setPosition(armInPos);
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .setReversed(true)
                .lineTo(new Vector2d(23.5,-35),
                        SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .setReversed(false)
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    slidePositionTarget= 700;
                })
                .UNSTABLE_addTemporalMarkerOffset(3,()->{
                    armLeft.setPosition(armOutPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(4,()->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .lineToSplineHeading(new Pose2d(50.8,-47.2, Math.toRadians(181)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    armLeft.setPosition(armInPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5,()->{
                    slidePositionTarget = 0.0;
                })
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToLinearHeading(new Pose2d(58.7,-11, Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();*/

        purplePixel1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(3.2, -35, Math.toRadians(300)),
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
                .addTemporalMarker(0.8,0,()->{
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
                .addDisplacementMarker(0.8,0,()->{
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
                .addDisplacementMarker(0.8,0,()->{
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

        //parks
        park1 = drive.trajectoryBuilder(reset1.end())
                .splineToLinearHeading(new Pose2d(58.7,-11, Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0.8,0,()->{
                    intakeLeft.setPosition(intakeDownPos);
                    intakeRight.setPosition(intakeDownPos);
                })
                .build();

        park2 = drive.trajectoryBuilder(reset2.end())
                .splineToLinearHeading(new Pose2d(58.7,-11, Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        park3 = drive.trajectoryBuilder(reset3.end())
                .lineToLinearHeading(new Pose2d(58.7,-11, Math.toRadians(182)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
    }
}