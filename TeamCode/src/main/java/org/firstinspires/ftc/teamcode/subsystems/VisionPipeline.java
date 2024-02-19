package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.AutoBlueScorePurple;
import org.firstinspires.ftc.teamcode.CenterStageOpMode;
import org.opencv.core.*; // TODO: make sure imports are right
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.*;

import java.util.Arrays;
@Config
public class VisionPipeline extends OpenCvPipeline {
  public static int x0 = 95, y0 = 90, w0 = 60 , h0 = 55, x1 = 240, y1 = 95, w1 = 55, h1 = 45, x2 = 382, y2 = 90, w2 = 50, h2 = 55;
  private int output = -1;
  private String telemestring;
  private boolean isBlue;
  private boolean isOutputSide;

  private boolean isHardToSeeRightSpot;

  private Scalar lowBlue =  new Scalar(105,  155, 35);
  private Scalar highBlue = new Scalar(130, 255, 255);
//  private Scalar lowRed2 =  new Scalar(164, 165, 55);
//  private Scalar highRed2 = new Scalar(179, 255, 255);
  private Scalar lowRed1 =  new Scalar(5, 165, 55);
  private Scalar highRed1 = new Scalar(30, 255, 255);

  private Mat blurred = new Mat();
  private Mat hsv = new Mat();
  private Mat threshold = new Mat();
  private Mat otherThreshold = new Mat(); // in case of red where 2 are needed
  private Mat leftROI = new MatOfDouble();
  private Mat midROI = new MatOfDouble();
  private Mat rightROI = new MatOfDouble();

  private double[] means = new double[3];

  public VisionPipeline(boolean blue, boolean outputSide, boolean hardToSeeRightSpot) {
    isBlue = blue;
    isOutputSide = outputSide;
    isHardToSeeRightSpot = hardToSeeRightSpot;
  }
   
  @Override
  public Mat processFrame(Mat input) {

    if(isOutputSide) {
      // frame processing for reading team prop
      Imgproc.GaussianBlur(input, blurred, new Size(15, 15), 0, 0);
      Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_BGR2HSV);
      
      if(isBlue) {
        Core.inRange(hsv, lowRed1, highRed1, threshold);
//        Core.inRange(hsv, lowRed2, highRed2, otherThreshold);
//        Core.bitwise_or(threshold, otherThreshold, threshold);
      } else {
        Core.inRange(hsv, lowBlue, highBlue, threshold);
      }

      if(isHardToSeeRightSpot) {
        // either blue backdrop or red stack
        leftROI = threshold.submat(new Rect(x0, y0, w0, h0));
        midROI = threshold.submat(new Rect(x1, y1, w1, h1));
        rightROI = threshold.submat(new Rect(x2, y2, w2, h2));
      } else {
        // either blue stack or red backdrop
        leftROI = threshold.submat(new Rect(432 - (x2 + w2), y0, w2, h0));
        midROI = threshold.submat(new Rect(432 - (x1 + w1), y1, w1, h1));
        rightROI = threshold.submat(new Rect(432 - (x0 + w0), y2, w0, h2));
}

      Imgproc.rectangle(input, new Point(x0, y0), new Point(x0 + w0, y0 + h0), new Scalar(255, 0, 0), 5);
      Imgproc.rectangle(input, new Point(x1, y1), new Point(x1 + w1, y1 + h1), new Scalar(255, 0, 0), 5);
      Imgproc.rectangle(input, new Point(x2, y2), new Point(x2 + w2, y2 + h2), new Scalar(255, 0, 0), 5);

      Imgproc.rectangle(input, new Point(432 - (x2 + w2), y0), new Point(432 - (x2 + w2) + w2, y0 + h0), new Scalar(0, 255, 0), 5);
      Imgproc.rectangle(input, new Point(432 - (x1 + w1), y1), new Point(432 - (x1 + w1) + w1, y1 + h1), new Scalar(0, 255, 0), 5);
      Imgproc.rectangle(input, new Point(432 - (x0 + w0), y2), new Point(432 - (x0 + w0) + w0, y2 + h2), new Scalar(0, 255, 0), 5);

      means[0] = Core.sumElems(leftROI).val[0];
      means[1] = Core.sumElems(midROI).val[0];
      means[2] = Core.sumElems(rightROI).val[0];


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

      return threshold;
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
    //return telemestring;
      return "hello";
  }
}
