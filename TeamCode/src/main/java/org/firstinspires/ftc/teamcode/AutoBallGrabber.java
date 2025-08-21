package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.openftc.easyopencv.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "FollowRedBallMecanum", group = "Linear Opmode")
public class AutoBallGrabber extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo claw;

    private OpenCvWebcam webcam;
    private RedObjectPipeline pipeline;

    // Camera center (for centering error calculation)
    private static final int FRAME_WIDTH = 640;
    private static final int FRAME_HEIGHT = 480;
    private static final int CENTER_X = FRAME_WIDTH / 2;

    // Movement tuning constants — tune these for your robot
    private static final double STRAFE_KP = 0.0025; // Strafing proportional gain
    private static final double DRIVE_KP  = 0.025;  // Forward proportional gain
    private static final double MAX_STRAFE = 0.6;
    private static final double MAX_DRIVE  = 0.6;
    private static final int    CLOSE_RADIUS = 80;    // Radius (px) at which we stop and grab

    // claw positions - adjust for your servo
    private static final double CLAW_OPEN   = 0.2;
    private static final double CLAW_CLOSED = 0.8;

    @Override
    public void runOpMode() {
        // Map motors - change names to match your hardware config if needed
        frontLeft  = hardwareMap.get(DcMotor.class, "fRight");
        frontRight = hardwareMap.get(DcMotor.class, "fLeft");
        backLeft   = hardwareMap.get(DcMotor.class, "bRight");
        backRight  = hardwareMap.get(DcMotor.class, "bLeft");

        // Reverse right side motors for typical mecanum wiring (flip if needed)
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Map claw
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_OPEN);

        // Vision setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // NOTE: change "Webcam" to "Webcam 1" if your config uses that name
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        pipeline = new RedObjectPipeline();
        webcam.setPipeline(pipeline);

        telemetry.addLine("Opening camera...");
        telemetry.update();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("Camera opened, starting streaming");
                telemetry.update();
                webcam.showFpsMeterOnViewport(true);
                webcam.startStreaming(FRAME_WIDTH, FRAME_HEIGHT, OpenCvCameraRotation.UPRIGHT);
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

        boolean grabbed = false;

        while (opModeIsActive()) {
            double ballX = pipeline.getBallX();        // center x in pixels (0..FRAME_WIDTH) or -1 if none
            double ballRadius = pipeline.getBallRadius(); // 0 if none

            telemetry.addData("Ball X", ballX);
            telemetry.addData("Ball Radius", ballRadius);

            if (ballRadius > 0 && !grabbed) { // we see the ball and haven't grabbed it yet
                double errorX = ballX - CENTER_X; // positive -> ball is to the right

                // Strafing (left/right)
                double strafe = errorX * STRAFE_KP;
                strafe = Range.clip(strafe, -MAX_STRAFE, MAX_STRAFE);

                // Forward/backward based on radius (bigger radius = closer)
                double areaError = CLOSE_RADIUS - ballRadius; // positive when far
                double drive = areaError * DRIVE_KP;
                drive = Range.clip(drive, -MAX_DRIVE, MAX_DRIVE);

                // If close enough, stop and grab
                if (ballRadius >= CLOSE_RADIUS) {
                    stopMotors();
                    claw.setPosition(CLAW_CLOSED);
                    grabbed = true;
                    telemetry.addLine("Grabbed!");
                } else {
                    mecanumDrive(drive, strafe, 0.0);
                }
            } else {
                // no ball seen or already grabbed -> stop
                stopMotors();
            }

            telemetry.update();
            sleep(30);
        }

        // cleanup
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    private void mecanumDrive(double forward, double strafe, double rotate) {
        // combine
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        // normalize if any power > 1
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // ----------------- Pipeline (included so RedObjectPipeline is defined) -----------------
    public static class RedObjectPipeline extends OpenCvPipeline {
        private final Mat hsv = new Mat();
        private final Mat mask1 = new Mat();
        private final Mat mask2 = new Mat();
        private final Mat mask = new Mat();
        private final Mat hierarchy = new Mat();

        // outputs (volatile so OpMode thread sees updates)
        private volatile double ballX = -1;      // center x in px, -1 means not found
        private volatile double ballRadius = 0;  // radius in px (approximated by half max(width,height))

        public double getBallX() { return ballX; }
        public double getBallRadius() { return ballRadius; }

        @Override
        public Mat processFrame(Mat input) {
            // convert to HSV
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // define red ranges (two ranges because red wraps hue)
            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), mask1);
            Core.inRange(hsv, new Scalar(170, 100, 100), new Scalar(180, 255, 255), mask2);
            Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);

            // find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxArea = 0;
            Rect best = null;

            for (MatOfPoint c : contours) {
                Rect r = Imgproc.boundingRect(c);
                double a = r.area();
                if (a > 50) { // ignore tiny blobs
                    Imgproc.rectangle(input, r.tl(), r.br(), new Scalar(0, 255, 255), 1); // debug boxes
                    if (a > maxArea) {
                        maxArea = a;
                        best = r;
                    }
                }
            }

            if (best != null) {
                Imgproc.rectangle(input, best.tl(), best.br(), new Scalar(255, 0, 0), 2);
                ballX = best.x + best.width / 2.0;
                ballRadius = Math.max(best.width, best.height) / 2.0;
            } else {
                ballX = -1;
                ballRadius = 0;
            }

            return input;
        }
    }
}
