package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
public abstract class CenterStageAuto extends CenterStageOpMode {

    enum State {
        SCANNING,
        PURPLE,
        YELLOW,
        CYCLE,
        PARK
    }

    State currentState = State.PURPLE;
    VisionPipeline pipeline;

    @Override
    public void init() {
        super.init();

        pipeline = new VisionPipeline(red, true);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //error?
            }
        });


    }

    public void start() {
        switch (pipeline.getOutput()) {
            case 0:
                telemetry.addData("vision", "left");
                break;
            case 1:
                telemetry.addData("vision", "center");
                break;
            default:
                telemetry.addData("vision", "right");
                break;
        }
    }

    @Override
    public void childLoop() {
        drive.update();
        switch(currentState) {

        }
    }
}
