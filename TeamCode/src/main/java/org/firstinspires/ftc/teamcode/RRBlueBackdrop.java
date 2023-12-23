
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

    public static double slidePositionTarget = 100.0;
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
    Trajectory moveUp1;
    Trajectory moveUp2;
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
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        slidesPidRight = new SlidesPID();
        slidesPidLeft = new SlidesPID();

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

        armLeft.scaleRange(0.0, 0.245);

        pRight.scaleRange(0.68, 0.77);
        pLeft.scaleRange(0.57,0.67);

        /*rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE); */
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(16.4, 63.25,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //purple pixels
        purplePixel1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(23.5, 33.5),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    drive.followTrajectory(moveUp1);
                })
                .build();

        purplePixel2 = drive.trajectoryBuilder(startPose, true)
                .lineTo(new Vector2d(9,32),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    drive.followTrajectory(moveUp2);
                })
                .build();

        purplePixel3 = drive.trajectoryBuilder(startPose, true)
                .lineToLinearHeading(new Pose2d(4.8,32.71, Math.toRadians(60)),
                        SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    drive.followTrajectory(yellowPixel3);
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

        //yellow pixels
        yellowPixel1 = drive.trajectoryBuilder(moveUp1.end())
                .lineToLinearHeading(new Pose2d(50.2,32, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0.5,0, () ->{
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .addDisplacementMarker(()->{
                    rs.setPower(slidesPidRight.calculatePower(slidePositionTarget));
                    ls.setPower(slidesPidLeft.calculatePower(slidePositionTarget));
                })
                .addDisplacementMarker(() ->{
                    armLeft.setPosition(armOutPos);
                })
                .addDisplacementMarker(() ->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .addDisplacementMarker(()->{
                    drive.followTrajectory(reset1);
                })
                .build();

        yellowPixel2 = drive.trajectoryBuilder(moveUp2.end())
                .lineToLinearHeading(new Pose2d(50.2,38, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0.5,0, () ->{
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .addDisplacementMarker(() ->{
                    rs.setPower(slidesPidRight.calculatePower(slidePositionTarget));
                    ls.setPower(slidesPidLeft.calculatePower(slidePositionTarget));
                })
                .addDisplacementMarker(() ->{
                    armLeft.setPosition(armOutPos);
                })
                .addDisplacementMarker(() ->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .addDisplacementMarker(()->{
                    drive.followTrajectory(reset2);
                })
                .build();

        yellowPixel3 = drive.trajectoryBuilder(purplePixel3.end())
                .lineToSplineHeading(new Pose2d(50.2,44, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0.5,0, () ->{
                    pLeft.setPosition(plungerGrabPos);
                    pRight.setPosition(plungerGrabPos);
                })
                .addDisplacementMarker(() ->{
                    rs.setPower(slidesPidRight.calculatePower(slidePositionTarget));
                    ls.setPower(slidesPidLeft.calculatePower(slidePositionTarget));
                })
                .addDisplacementMarker(() ->{
                    armLeft.setPosition(armOutPos);
                })
                .addDisplacementMarker(() ->{
                    pLeft.setPosition(plungerReleasePos);
                    pRight.setPosition(plungerReleasePos);
                })
                .addDisplacementMarker(()->{
                    drive.followTrajectory(reset3);
                })
                .build();

                //resets
                reset1 = drive.trajectoryBuilder(yellowPixel1.end())
                        .forward(10,
                                SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .addDisplacementMarker(()->{
                            armLeft.setPosition(armInPos);
                        })

                        .addDisplacementMarker(()->{
                            rs.setPower(slidesPidRight.calculatePower(0.0));
                            ls.setPower(slidesPidLeft.calculatePower(0.0));
                        })
                        .addDisplacementMarker(()->{
                            drive.followTrajectory(park1);
                        })
                        .build();

                reset2 = drive.trajectoryBuilder(yellowPixel2.end())
                        .forward(10,
                                SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .addDisplacementMarker(()->{
                            armLeft.setPosition(armInPos);
                        })
                        .addDisplacementMarker(()->{
                            rs.setPower(slidesPidRight.calculatePower(0.0));
                            ls.setPower(slidesPidLeft.calculatePower(0.0));
                        })
                        .addDisplacementMarker(()->{
                            drive.followTrajectory(park2);
                        })
                        .build();

                reset3 = drive.trajectoryBuilder(yellowPixel3.end())
                        .forward(10,
                                SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .addDisplacementMarker(()->{
                            armLeft.setPosition(armInPos);
                        })
                        .addDisplacementMarker(()->{
                            rs.setPower(slidesPidRight.calculatePower(0.0));
                            ls.setPower(slidesPidLeft.calculatePower(0.0));
                        })
                        .addDisplacementMarker(()->{
                            drive.followTrajectory(park3);
                        })
                        .build();

                        //parks
        park1 = drive.trajectoryBuilder(reset1.end())
                .splineToConstantHeading(new Vector2d(58.7,11),0,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//
                    lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rf.setPower(0);
                    lf.setPower(0);
                    rb.setPower(0);
                    lb.setPower(0);
                    ls.setPower(0);
                    rs.setPower(0);
                    intake.setPower(0);
                })
                .build();

        park2 = drive.trajectoryBuilder(reset2.end())
                .splineToConstantHeading(new Vector2d(58.7,11),0,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rf.setPower(0);
                    lf.setPower(0);
                    rb.setPower(0);
                    lb.setPower(0);
                    ls.setPower(0);
                    rs.setPower(0);
                    intake.setPower(0);
                })
                .build();

        park3 = drive.trajectoryBuilder(reset3.end())
                .splineToConstantHeading(new Vector2d(58.7,11),0,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rf.setPower(0);
                    lf.setPower(0);
                    rb.setPower(0);
                    lb.setPower(0);
                    ls.setPower(0);
                    rs.setPower(0);
                    intake.setPower(0);
                })
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
            break;
        }
    }
    @Override
    public void start(){
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void loop(){
        timeGap = timer.milliseconds();
        timer.reset();
        slidesPidRight.update(rs.getCurrentPosition(), timeGap);
        slidesPidLeft.update(ls.getCurrentPosition(), timeGap);
        drive.update();

        telemetry.addData("idkkkkkkk",slidesPidLeft.calculatePower(slidePositionTarget));
        telemetry.addData("pRight", "Position: " + pRight.getPosition());
        telemetry.addData("pLeft", "Position: " + pLeft.getPosition());
        telemetry.addData("armLeft", "Position: " + armLeft.getPosition());
        telemetry.addData("slides position rs: ", "current rs pos: " + rs.getCurrentPosition());
        telemetry.addData("slides position ls: ", "current ls pos: " + ls.getCurrentPosition());
        telemetry.addData("slides power rs: ", "current rs power: " + rs.getPower());
        telemetry.addData("slides power ls: ", "current ls power: " + ls.getPower());
        telemetry.update();
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