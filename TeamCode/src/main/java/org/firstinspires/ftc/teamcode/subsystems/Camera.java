//----------------------------------------------------------------- Camera.java -----------------------------------------------------------------
import org.opencv.core.*; // TODO: make sure imports are right

public class Camera {

  private OpenCvWebcam webcam;
  private HardwareMap hardwareMap;
  private VisionPipeline teamPropPipeline;

  public Camera(HardwareMap hw, boolean isRedAlliance, boolean isOutputSideCamera) { // hardware map from the base class is a parameter
    teamPropPipeline = new VisionPipeline(isRedAlliance, isOutputSideCamera); 

    this.hardwareMap = hw; //Configure the Camera in hardwaremap
    int cameraMonitorViewId =
        hardwareMap
            .appContext
            .getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // TODO: replace with actual id

    // TODO: get camera from hardware map, replace "camera" with what is in controlhub
    webcam =
        OpenCvCameraFactory.getInstance()
            .createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
    
    webcam.setPipeline(teamPropPipeline); // Setting the intial pipeline
    
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
