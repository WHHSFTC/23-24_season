
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
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous (preselectTeleOp = "CenterStageTele")
public class RRBlueBackdrop extends OpMode{

    FtcDashboard dashboard;
    TelemetryPacket packet;
    //public static MultipleTelemetry dashTelemetry = new MultipleTelemetry();

    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;
    DcMotor intake;
    DcMotor ls;
    DcMotor rs;

    public static int slidePositionTarget = 300;
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
    public static int vision = 1;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    SlidesPID slidesPidRight;
    SlidesPID slidesPidLeft;
    double timeGap = 0.0;
    boolean intakeOnGround;

    //Servo armRight;
    Servo armLeft;
    Servo pRight;
    Servo pLeft;
    Servo intakeRight;
    Servo intakeLeft;

    TouchSensor slidesLimit;
    DistanceSensor rightDS;
    DistanceSensor leftDS;
    SampleMecanumDrive drive;
    Trajectory purplePixel1;
    Trajectory purplePixel2;
    Trajectory purplePixel3;
    Trajectory yellowPixel1;
    Trajectory yellowPixel2;
    Trajectory yellowPixel3;
    Trajectory park1;
    Trajectory park2;
    Trajectory park3;

    @Override
    public void init(){
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        rf = hardwareMap.get(DcMotor.class, "motorRF");
        lf = hardwareMap.get(DcMotor.class, "motorLF");
        rb = hardwareMap.get(DcMotor.class, "motorRB");
        lb = hardwareMap.get(DcMotor.class, "motorLB");
        intake = hardwareMap.get(DcMotor.class, "motorIntake");
        ls = hardwareMap.get(DcMotor.class, "motorLS");
        rs = hardwareMap.get(DcMotor.class, "motorRS");

        armLeft = hardwareMap.get(Servo.class, "armLeft");
        //armRight = hardwareMap.get(Servo.class, "armRight");
        pRight = hardwareMap.get(Servo.class, "plungerRight");
        pLeft = hardwareMap.get(Servo.class, "plungerLeft");

        slidesLimit = hardwareMap.get(TouchSensor.class, "slidesLimit");
        rightDS = hardwareMap.get(DistanceSensor.class, "rightDS");
        leftDS = hardwareMap.get(DistanceSensor.class, "leftDS");

        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);
        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");

        intakeLeft.scaleRange(0.0, 1.0);
        intakeRight.scaleRange(0.0, 1.0);

        intakeRight.setDirection(Servo.Direction.REVERSE);

        // intakeLeft.setPosition(1.0);
        // intakeRight.setPosition(1.0);

        armLeft.scaleRange(0.0, 0.245);
        //armRight.scaleRange(0.0, 0.245);

        pRight.scaleRange(0.68, 0.77);
        pLeft.scaleRange(0.57,0.67);

        /*rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE); */
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(16.4, 63.25,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        purplePixel1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(23.5, 33.5),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        purplePixel2 = drive.trajectoryBuilder(startPose, true)
                .lineTo(new Vector2d(9,31.5),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        purplePixel3 = drive.trajectoryBuilder(startPose, true)
                .lineToLinearHeading(new Pose2d(4.8,32.71, Math.toRadians(60)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker( () -> {
                    drive.followTrajectory(yellowPixel3);
                })
                .build();

        yellowPixel3 = drive.trajectoryBuilder(purplePixel3.end())
                .lineToSplineHeading(new Pose2d(50.2,44, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(48, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0.5,0, () ->{
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .addDisplacementMarker(() ->{
                    //rs.setPower(SlidesPID.calculatePower());
                    //ls.setPower(SlidesPID.calculatePower());
                })
                .addDisplacementMarker(() ->{
                    armLeft.setPosition(armOutPos);
                })
                .addDisplacementMarker(() ->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .addDisplacementMarker(() ->{
                    drive.followTrajectory(park3);
                })
                .build();

                park3 = drive.trajectoryBuilder(yellowPixel3.end())
                        .lineToLinearHeading(new Pose2d(43.2,44, Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .addDisplacementMarker(()->{
                            armLeft.setPosition(armInPos);
                        })
                        .addDisplacementMarker(()->{
                            //bring slides all the way down
                        })
                        .splineToConstantHeading(new Vector2d(60.7,12.3),0)
                        .build();

        intakeLeft.setPosition(intakeUpPos);
        intakeRight.setPosition(intakeUpPos);

        switch (vision) {
            case 1:
                drive.followTrajectoryAsync(purplePixel1);
                break;
            case 2:
                drive.followTrajectoryAsync(purplePixel2);
                break;
            case 3:
                drive.followTrajectoryAsync(purplePixel3);

                drive.followTrajectoryAsync(park3);
                break;
        }
    }

    @Override
    public void start(){

    }
    @Override
    public void loop(){
        timeGap = timer.milliseconds();
        timer.reset();
        slidesPidRight.update(rs.getCurrentPosition(), timeGap);
        slidesPidLeft.update(ls.getCurrentPosition(), timeGap);
    }
    public void stop(){
        super.stop();
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        ls.setPower(0);
        rs.setPower(0);
    }
}