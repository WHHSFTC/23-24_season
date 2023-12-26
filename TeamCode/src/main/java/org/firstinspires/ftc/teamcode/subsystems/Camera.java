package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Camera {

  private OpenCvWebcam webcam;
  private HardwareMap harrdwareMap;
  public VisionPipeline teamPropPipeline;

  public Camera(HardwareMap hw, boolean isRedAlliance, boolean isOutputSideCamera, String name) { // hardware map from the base class is a parameter
    teamPropPipeline = new VisionPipeline(isRedAlliance, isOutputSideCamera); 

    this.harrdwareMap = hw; //Configure the Camera in hardwaremap
    int cameraMonitorViewId =
        harrdwareMap
            .appContext
            .getResources()
            .getIdentifier("cameraMonitorViewId", "id", harrdwareMap.appContext.getPackageName()); // TODO: replace with actual id

    webcam =
        OpenCvCameraFactory.getInstance()
            .createWebcam(harrdwareMap.get(WebcamName.class, name), cameraMonitorViewId);
    
    webcam.setPipeline(teamPropPipeline); // Setting the initial pipeline
    
    webcam.setMillisecondsPermissionTimeout(2500);
    
    // Streaming Frames
    webcam.openCameraDeviceAsync(
        new OpenCvCamera.AsyncCameraOpenListener() {
          @Override
          public void onOpened() {
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
          }

          @Override
          public void onError(int errorCode) {
	    // to be called when an error happens 
	  }
        });
  }
  
  
  // Get information from pipeline 
  public int getPipelineOutput(){ 
      return teamPropPipeline.getOutput(); 
  }
  
  // call stop at the end of the opMode. 
  public void stop() {
    webcam.stopStreaming();
  }
}
