package org.firstinspires.ftc.teamcode

//------------------------------------------------------------- VisionPipeline.java -------------------------------------------------------------
import org.opencv.core.*; // TODO: make sure imports are right
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class VisionPipeline extends OpenCvPipeline {
   
  private int output = -1;
  private bool isRed;
  private bool isOutputSide;

  private Scalar lowBlue =  new Scalar(95,  165, 55);
  private Scalar highBlue = new Scalar(125, 255, 255);
  private Scalar lowRed2 =  new Scalar(164, 165, 55);
  private Scalar highRed2 = new Scalar(179, 255, 255);
  private Scalar lowRed1 =  new Scalar(0,   165, 55);
  private Scalar highRed1 = new Scalar(5,   255, 255);

  private Mat blurred = new Mat();
  private Mat hsv = new Mat();
  private Mat threshold = new Mat();
  private Mat otherThreshold = new Mat(); // in case of red where 2 are needed

  public VisionPipeline(boolean red, boolean outputSide) {
    isRed = red;
    isOutputSide = outputSide;
  }
   
  @Override
  public Mat processFrame(Mat input) {
    if(isOutputSide) {
      // frame processing for reading team prop
      Imgproc.GaussianBlur(input, blurred, new Size(15, 15), 0, 0);
      Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_BGR2HSV);
      
      if(isRed) {	
        Core.inRange(hsv, lowRed1, highRed1, threshold);
        Core.inRange(hsv, lowRed2, highRed2, otherThreshold);
        Core.bitwise_or(threshold, otherThreshold, threshold);
      } else {
        Core.inRange(hsv, lowBlue, highBlue, threshold);
      }      
    } else {
      // frame processing for aligning to stack

    }
    
    // TODO: finish frame processing here and set output
    output = (int)(Math.random()*3); 
    return input;
  }

  public int getOutput() {
    return output;
  }
}
