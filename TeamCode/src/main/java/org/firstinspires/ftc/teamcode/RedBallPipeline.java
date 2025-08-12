package org.firstinspires.ftc.teamcode;

import android.icu.text.UFormat;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Largest Red Object Finder", group = "Vision")
public class RedBallPipeline extends LinearOpMode {

    OpenCvWebcam webcam;
    RedPipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        pipeline = new RedPipeline();
        webcam.setPipeline(pipeline);

        telemetry.addLine("Opening camera...");
        telemetry.update();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("Camera opened, starting streaming");
                telemetry.update();

                webcam.showFpsMeterOnViewport(true);
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera open error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Red Objects Found", pipeline.numRedObjects);

            if (pipeline.largestRect != null) {
                Rect r = pipeline.largestRect;
                telemetry.addData("Largest Object Area", "%.1f", r.area());
                telemetry.addData("Largest Object Pos", "x=%d y=%d", r.x, r.y);
                telemetry.addData("Largest Object Size", "w=%d h=%d", r.width, r.height);
            } else {
                telemetry.addData("Largest Object", "None detected");
            }

            telemetry.update();
            sleep(50);
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    public static class RedPipeline extends OpenCvPipeline {

        Mat hsv = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat mask = new Mat();
        Mat hierarchy = new Mat();

        public int numRedObjects = 0;
        public Rect largestRect = null;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), mask1);
            Core.inRange(hsv, new Scalar(170, 100, 100), new Scalar(180, 255, 255), mask2);

            Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            numRedObjects = 0;
            largestRect = null;
            double maxArea = 0;

            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                double area = rect.area();
                if (area > 50) {
                    Imgproc.rectangle(input, rect, new Scalar(0, 255, 255), 1);
                    numRedObjects++;

                    if (area > maxArea) {
                        maxArea = area;
                        largestRect = rect;
                    }
                }
            }

            if (largestRect != null) {
                Imgproc.rectangle(input, largestRect.tl(), largestRect.br(), new Scalar(255, 0, 0), 2);
            }

            return input;
        }
    }
}
