package org.firstinspires.ftc.teamcode.subsystems;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.AutoBlueScorePurple;
import org.firstinspires.ftc.teamcode.CenterStageOpMode;
import org.opencv.core.*; // TODO: make sure imports are right
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.*;

import java.util.Arrays;

public class VisionPipeline extends OpenCvPipeline {
   
  private int output = -1;
  private String telemestring;
  private boolean isBlue;
  private boolean isOutputSide;

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
  private Mat leftROI = new MatOfDouble();
  private Mat midROI = new MatOfDouble();
  private Mat rightROI = new MatOfDouble();

  private double[] means = new double[3];

  public VisionPipeline(boolean blue, boolean outputSide) {
    isBlue = blue;
    isOutputSide = outputSide;
  }
   
  @Override
  public Mat processFrame(Mat input) {

    if(isOutputSide) {
      // frame processing for reading team prop
      Imgproc.GaussianBlur(input, blurred, new Size(15, 15), 0, 0);
      Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_BGR2HSV);
      
      if(isBlue) {	
        Core.inRange(hsv, lowRed1, highRed1, threshold);
        Core.inRange(hsv, lowRed2, highRed2, otherThreshold);
        Core.bitwise_or(threshold, otherThreshold, threshold);
      } else {
        Core.inRange(hsv, lowBlue, highBlue, threshold);
      }
      leftROI = threshold.submat(new Rect(0, 0, 80, 240));
      midROI = threshold.submat(new Rect(80, 0, 160, 240));
      rightROI = threshold.submat(new Rect(240, 0, 80, 240));

      means[0] = Core.mean(leftROI).val[0];
      means[1] = Core.mean(midROI).val[0];
      means[2] = Core.mean(rightROI).val[0];

      if(means[0] > means[1]) {
        if(means[0] > means[2]) {
          output = 0;
        } else {
          output = 2;
        }
      } else if(means[1] > means[2]) {
        output = 1;
      }
      else if(means[2] > means[1]){
        output = 2;
      }

    } else {
      // frame processing for aligning to stack
      output = -2;
    }

    // TODO: finish frame processing here and set output
    return threshold;
  }

  public int getOutput() {
    return output;
  }
  public String getPipelineTelemetry() {
//    telemestring = means[0] + " (L), " + means[1] + " (M), " + means[2] + " (R).";
    telemestring = "AAHHHHHHHH LOOK" + Core.mean(leftROI) + " (L), " + Core.mean(midROI) + " (M), " + Core.mean(rightROI) + " (R).";
    return telemestring;
  }
}
