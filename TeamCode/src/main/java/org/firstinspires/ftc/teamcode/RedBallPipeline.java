package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Red Object Finder", group="Vision")
public class RedBallPipeline extends LinearOpMode {

    OpenCvWebcam webcam;
    RedPipeline pipeline;

    @Override
    public void runOpMode() {
        // Get camera monitor view ID
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Link to the webcam in your RC config (named "Webcam 1")
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam"),
                cameraMonitorViewId
        );

        // Create and set the pipeline
        pipeline = new RedPipeline();
        webcam.setPipeline(pipeline);

        // Open the camera and start streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Report how many red objects found
            telemetry.addData("Red objects found", pipeline.numRedObjects);
            telemetry.update();
            sleep(50); // Prevent spam
        }
    }

    // ===================== RED DETECTION PIPELINE =====================
    public static class RedPipeline extends OpenCvPipeline {
        Mat hsv = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat mask = new Mat();
        Mat hierarchy = new Mat();
        public int numRedObjects = 0;

        @Override
        public Mat processFrame(Mat input) {
            // Convert RGB to HSV
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // First red range
            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), mask1);
            // Second red range
            Core.inRange(hsv, new Scalar(170, 100, 100), new Scalar(180, 255, 255), mask2);

            // Combine masks
            Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            numRedObjects = 0;

            // Draw rectangles around red objects
            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                if (rect.area() > 500) { // Filter out small noise
                    Imgproc.rectangle(input, rect, new Scalar(0, 255, 0), 2);
                    numRedObjects++;
                }
            }

            return input; // Display frame with rectangles
        }
    }
}
