
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
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.opencv.core.Mat;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous (preselectTeleOp = "CenterStageTele")
public class RRRedBackdrop extends CenterStageAuto{

    FtcDashboard dashboard;
    TelemetryPacket packet;
    //public static MultipleTelemetry dashTelemetry = new MultipleTelemetry();

    public static double slidesff = 0.0;
    public static double slideTargetGain = 100.0;
    public static double slideMin = 0.0;
    public static double slideMax = 2200.0;
    boolean slidesPressed;
    boolean dpadDownPressed;
    double slideSavedPosition = 1100.0;
    public static double intakeUpPos = 0.64;
    public static double intakeDownPos = 0.07;
    public static double intakeStackPos = 0.18;
    public static double armOutPos = 0.1;
    public static double armInPos = 1.0;
    public static double plungerGrabPos = 0.0;
    public static double plungerReleasePos = 1.0;
    public static double dronePos1 = 0.35;
    public static double dronePos2 = 0.95;
    double timeGap = 0.0;
    boolean intakeOnGround;

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
                .addDisplacementMarker(()->{
                    drive.followTrajectory(moveUp1);
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
                .addDisplacementMarker(()->{
                    drive.followTrajectory(moveUp2);
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
                .addDisplacementMarker(()->{
                    drive.followTrajectory(moveUp3);
                })
                .build();

        //move up
        moveUp1 = drive.trajectoryBuilder(purplePixel1.end())
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    drive.followTrajectory(yellowPixel1);
                })
                .build();

        moveUp2 = drive.trajectoryBuilder(purplePixel2.end())
                .forward(5, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    drive.followTrajectory(yellowPixel2);
                })
                .build();
        moveUp3 = drive.trajectoryBuilder(purplePixel3.end())
                .forward(5, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    drive.followTrajectory(yellowPixel3);
                })
                .build();
        //yellow pixels
        yellowPixel1 = drive.trajectoryBuilder(moveUp1.end())
                .lineToLinearHeading(new Pose2d(50.8,-34.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                /*.addDisplacementMarker(0.3,0,()->{
                    slidePositionTarget = 400.0;
                })*/
                .addDisplacementMarker(0.8,0,()->{
                    armLeft.setPosition(armOutPos);
                })
                .addDisplacementMarker(()->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .addDisplacementMarker(()->{
                    drive.followTrajectory(reset1);
                })
                .build();

        yellowPixel2 = drive.trajectoryBuilder(moveUp2.end())
                .lineToLinearHeading(new Pose2d(50.8,-42, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                /*.addDisplacementMarker(0.3,0,()->{
                    slidePositionTarget = 400.0;
                })*/
                .addDisplacementMarker(0.8,0,()->{
                    armLeft.setPosition(armOutPos);
                })
                .addDisplacementMarker(()->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .addDisplacementMarker(()->{
                    drive.followTrajectory(reset2);
                })
                .build();

        yellowPixel3 = drive.trajectoryBuilder(moveUp3.end())
                .lineToSplineHeading(new Pose2d(50.8,-47.2, Math.toRadians(181)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0.8,0,()->{
                    armLeft.setPosition(armOutPos);
                })
                .addDisplacementMarker(()->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .addDisplacementMarker(()->{
                    drive.followTrajectory(reset3);
                })
                .build();

        //resets
        reset1 = drive.trajectoryBuilder(yellowPixel1.end())
                .forward(8,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0.2, 0, ()->{
                    armLeft.setPosition(armInPos);
                })
                .addDisplacementMarker(0.5,0,()->{
                    rs.setPower(slidesPidRight.calculatePower(0.0));
                    ls.setPower(slidesPidLeft.calculatePower(0.0));
                })
                .addDisplacementMarker(()->{
                    drive.followTrajectory(park1);
                })
                .build();

        reset2 = drive.trajectoryBuilder(yellowPixel2.end())
                .forward(8,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(0.2, 0, ()->{
                    armLeft.setPosition(armInPos);
                })
                .addDisplacementMarker(0.5,0,()->{
                    rs.setPower(slidesPidRight.calculatePower(0.0));
                    ls.setPower(slidesPidLeft.calculatePower(0.0));
                })
                .addDisplacementMarker(()->{
                    drive.followTrajectory(park2);
                })
                .build();

        reset3 = drive.trajectoryBuilder(yellowPixel3.end())
                .forward(10,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0.2, 0, ()->{
                    armLeft.setPosition(armInPos);
                })
                .addDisplacementMarker(0.5,0,()->{
                    rs.setPower(slidesPidRight.calculatePower(0.0));
                    ls.setPower(slidesPidLeft.calculatePower(0.0));
                })
                .addDisplacementMarker(()->{
                    drive.followTrajectory(park3);
                })
                .build();

        //parks
       /* park1 = drive.trajectoryBuilder(reset1.end())
                .splineToLinearHeading(new Pose2d(58.7,-11, Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.5, 0, ()->{
                    super.stop();
                })
                .build();

        park2 = drive.trajectoryBuilder(reset2.end())
                .splineToLinearHeading(new Pose2d(58.7,-11, Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.5, 0, ()->{
                    super.stop();
                })
                .build();

        park3 = drive.trajectoryBuilder(reset3.end())
                .lineToLinearHeading(new Pose2d(58.7,-11, Math.toRadians(182)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.5, 0, ()->{
                    super.stop();
                })
                .build();*/

        intakeLeft.setPosition(intakeUpPos);
        intakeRight.setPosition(intakeUpPos);
        pLeft.setPosition(plungerGrabPos);
        pRight.setPosition(plungerGrabPos);
        armLeft.setPosition(armInPos);
        droneLauncher.setPosition(1.0);
        slidePositionTarget = 700.0;
    }
    @Override
    public void start(){
        super.start();
        switch (elementPosition) {

            case 0:
                drive.followTrajectoryAsync(purplePixel1);
                break;
            case 1:
                drive.followTrajectoryAsync(purplePixel2);
                break;
            case 2:
                drive.followTrajectory(purplePixel3);
            default:
                drive.followTrajectoryAsync(purplePixel1);
                break;
        }
    }

    @Override
    public void childLoop() {
        super.childLoop();
        telemetry.addData("target", slidePositionTarget);
        telemetry.addData("loopvision", pipeline.getOutput());
    }
    public void stop(){
        super.stop();
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        ls.setPower(0);
        rs.setPower(0);
        intake.setPower(0);
    }
}