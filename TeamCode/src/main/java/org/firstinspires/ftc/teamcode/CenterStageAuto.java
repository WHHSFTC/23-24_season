package org.firstinspires.ftc.teamcode;

import static android.provider.SyncStateContract.Helpers.update;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
public abstract class CenterStageAuto extends CenterStageOpMode implements AutoInterface {
    int elementPosition;
    double delay = 0.0;
    double distancePower;
    double anglePower;
    boolean isBusy;

    enum AutoState {
        PURPLE,
        MOVEUP,
        YELLOW,
        RESET,
        TO_STACK,
        INTAKE,
        FROM_STACK,
        OUTPUT,
        PARK,
        IDLE
    }

    AutoState currentState;
    VisionPipeline pipeline;
    ElapsedTime liftTimer = new ElapsedTime();

    @Override
    public void init() {
        super.init();
        liftTimer.reset();
        slidePositionTarget = 0.0;
        pipeline = new VisionPipeline(blue, true, isRightSideHardForCameraToSee);
        webcam.setPipeline(pipeline);

        intakeLeft.setPosition(intakeUpPos);
        intakeRight.setPosition(intakeUpPos);
        pLeft.setPosition(plungerGrabPos);
        pRight.setPosition(plungerGrabPos);
        armLeft.setPosition(armInPos);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //error?
            }
        });
    }

    @Override
    final public void init_loop() {
        telemetry.addData("state", currentState);
        telemetry.addData("is blue auto?", blue);
        telemetry.addData("Hard to see right? (Should be true for BB and RS) ", isRightSideHardForCameraToSee);
        telemetry.addData("pipeline", pipeline.getPipelineTelemetry() + "     " + pipeline.getOutput());
        telemetry.addData("delay time", delay);

        telemetry.update();
//
        if(gamepad1.dpad_up && !gamepad1prev.dpad_up){
            delay += 500.0;
        }else if(gamepad1.dpad_down && !gamepad1prev.dpad_down){
            if(delay <= 500.0){
                delay = 0.0;
            }
            else{
                delay -= 500.0;
            }
        }else{
            delay = delay;
        }

        currentState = AutoState.PURPLE;
        telemetry.addData("delay time", delay);
        gamepad1prev.copy(gamepad1);
        gamepad2prev.copy(gamepad2);
    }

    @Override
    public void start() {
        //webcam.stopStreaming(); //TODO see if this is good
        ElapsedTime wait = new ElapsedTime();
        while(wait.milliseconds() < delay){
            telemetry.addData("delay time", wait.milliseconds());
            telemetry.update();
        }
        super.start();
        imu.resetYaw();
        elementPosition = pipeline.getOutput();
        followPurple();
    }


    @Override
    public void childLoop() {
        slidesPidLeft.update(ls.getCurrentPosition(),timePerLoop);
        slidesPidRight.update(rs.getCurrentPosition(),timePerLoop);
        rs.setPower(slidesPidRight.calculatePower(slidePositionTarget));
        ls.setPower(slidesPidLeft.calculatePower(slidePositionTarget));
        if(drive.isBusy()){
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
                    currentState = AutoState.PARK;
                    followPark();
                }
                break;
            case TO_STACK:
                if (!drive.isBusy()) {
                    currentState = AutoState.INTAKE;
                }
                break;
            case INTAKE:
                if (!drive.isBusy()) {
                    currentState = AutoState.OUTPUT;
                }
                break;
            case FROM_STACK:

                break;
            case OUTPUT:

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
    public void followPurple(){

    }

    public void followMOVEUP(){

    }
    public void followYellow(){

    }
    public void followReset(){

    }
    public void followToStack(){

    }
    public void followIntake(){

    }
    public void followFromStack(){

    }
    public void followOutput(){
        //might need to run without encoders

        double x, r;
        distancePower = DSpid.distancePID(rightDS.getDistance(DistanceUnit.INCH), leftDS.getDistance(DistanceUnit.INCH) - leftDSOffset, timePerLoop, distBackdrop);
        anglePower = DSpid.anglePID(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), timePerLoop, Math.toRadians(270));

        x = -distancePower;
        r = -anglePower;

        double preRF = 1.3*r + x;
        double preLF = 1.3*r - x;
        double preRB = 1.3*r + x;
        double preLB = 1.3*r - x;


        double max = Math.max(Math.max(Math.max(Math.max(preRF,preRB), preLB), preLF), 1);


        rf.setPower(preRF/max);
        lf.setPower(preLF/max);
        rb.setPower(preRB/max);
        lb.setPower(preLB/max);
    }
    public void followPark(){

    }
}
