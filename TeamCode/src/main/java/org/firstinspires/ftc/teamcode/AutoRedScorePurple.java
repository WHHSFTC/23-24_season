package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Disabled
@Autonomous
public class AutoRedScorePurple extends CenterStageAuto {

    Trajectory purplePixel1;
    Trajectory purplePixel2;
    Trajectory purplePixel3;

    public void init() {
        blue = false;
        super.init();
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(16.4, 63.25,Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        //TODO: change trajectory and remove @Disabled tag
        purplePixel1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(23.5, 33.5),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        purplePixel2 = drive.trajectoryBuilder(startPose, true)
                .lineTo(new Vector2d(9,32),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        purplePixel3 = drive.trajectoryBuilder(startPose, true)
                .lineToLinearHeading(new Pose2d(4.8,32.71, Math.toRadians(60)),
                        SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
    }

    @Override
    public void start() {
        super.start();
        switch (elementPosition) {
            case 0:
                drive.followTrajectoryAsync(purplePixel1);
                break;
            case 1:
                drive.followTrajectoryAsync(purplePixel2);
                break;
            default:
                drive.followTrajectoryAsync(purplePixel3);
                break;
        }
    }

    @Override
    public void childLoop() {
        super.childLoop();
        telemetry.addData("loopvision", pipeline.getOutput());
    }
}
