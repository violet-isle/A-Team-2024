package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.*;

@TeleOp
public class WebcamExample extends LinearOpMode {
    OpenCvWebcam webcam;
    BlueSampleHSV bluePipeline = new BlueSampleHSV();

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(bluePipeline);
        webcam.setMillisecondsPermissionTimeout(5000);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

            Point position = bluePipeline.getPosition();
            telemetry.addData("Pipeline Result", (position != null) ? position.toString() : "No target detected");

            telemetry.update();

            if (gamepad1.a) {
                webcam.stopStreaming();
            }

            sleep(100);
        }
    }
}



class BlueSampleHSV extends OpenCvPipeline {
   public Scalar lowerHSV = new Scalar(80, 100, 190, 0.0);
   public Scalar upperHSV = new Scalar(130.0, 255.0, 254.6, 255.0);
   public int kernelSize = 4;
   public int dilate = 0;
   public int erode = 1;
   private Mat hsvBinaryMat = new Mat();
   private Mat inputMask = new Mat();
   private Mat output = new Mat();
   private ArrayList<MatOfPoint> contours = new ArrayList<>();
   private Mat hierarchy = new Mat();
   public MatOfPoint biggestContour = null;
   private MatOfPoint2f points2f = new MatOfPoint2f();
   private ArrayList<RotatedRect> rotRects = new ArrayList<>();






   @Override
   public Mat processFrame(Mat input) {
       if (input.empty())
           return input;


       Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);


       inputMask.release();
       hsvBinaryMat.release();
       output.release();


       Core.inRange(input, lowerHSV, upperHSV, hsvBinaryMat);


       Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(kernelSize, kernelSize));


       for (int i = 0; i < erode; i++) {
           Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);
           Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);
           Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);
       }
       for (int i = 0; i < dilate; i++) {
           Imgproc.dilate(hsvBinaryMat, hsvBinaryMat, kernel);
           Imgproc.dilate(hsvBinaryMat, hsvBinaryMat, kernel);
           Imgproc.dilate(hsvBinaryMat, hsvBinaryMat, kernel);
       }


       Imgproc.erode(hsvBinaryMat, hsvBinaryMat, kernel);


       Core.bitwise_and(input, input, output, hsvBinaryMat);


       contours.clear();
       hierarchy.release();


       Imgproc.findContours(hsvBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


       // Clear any previous "biggestContour"
       biggestContour = null;


       // Desired ratio ~ 2.33 (3.5 : 1.5)
       double targetRatio = 3.5 / 1.5;


       // Variables to track best contour
       double bestRatioDiff = Double.MAX_VALUE;
       double bestArea = 0;
       MatOfPoint bestContour = null;


       for (MatOfPoint contour : contours) {
           if (contour == null)
               continue;


           double area = Imgproc.contourArea(contour);
           // Skip small contours
           if (area < 100)
               continue;


           // Convert to points2f to compute minAreaRect
           MatOfPoint2f tempPoints2f = new MatOfPoint2f();
           contour.convertTo(tempPoints2f, CvType.CV_32F);


           // Create the rotated rect
           RotatedRect rect = Imgproc.minAreaRect(tempPoints2f);


           // Compute ratio = max(width, height) / min(width, height)
           double w = rect.size.width;
           double h = rect.size.height;
           if (w < 1e-5 || h < 1e-5)
               continue;
           double ratio = Math.max(w, h) / Math.min(w, h);


           // Compare how close ratio is to targetRatio
           double ratioDiff = Math.abs(ratio - targetRatio);


           // If ratio is closer OR same ratio but bigger area => update best
           if (ratioDiff < bestRatioDiff) {
               bestRatioDiff = ratioDiff;
               bestArea = area;
               bestContour = contour;
           } else if (Math.abs(ratioDiff - bestRatioDiff) < 1e-5) {
               // ratio is effectively the same; check area
               if (area > bestArea) {
                   bestArea = area;
                   bestContour = contour;
               }
           }
       }


       // Update biggestContour to whichever is best
       biggestContour = bestContour;


       rotRects.clear();
       if (biggestContour != null && biggestContour.toArray().length >= 3) {
           biggestContour.convertTo(points2f, CvType.CV_32F);
           rotRects.add(Imgproc.minAreaRect(points2f));
       }


       Mat output = new Mat();
       Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 2);


       input.copyTo(output);


       for (RotatedRect rect : rotRects) {
           if (rect != null) {
               Point[] rectPoints = new Point[4];
               rect.points(rectPoints);
               Imgproc.polylines(output, Collections.singletonList(new MatOfPoint(rectPoints)), true,
                       new Scalar(0, 255, 0), 2);
           }
       }


       Imgproc.cvtColor(output, output, Imgproc.COLOR_HSV2RGB);
       if (!rotRects.isEmpty()){
           Imgproc.circle(output, rotRects.get(0).center, 6, new Scalar(0, 255, 0), -1);

       }
       return output;
   }


   public Point getPosition() {
       return (!rotRects.isEmpty()) ? rotRects.get(0).center : null;
   }
}




