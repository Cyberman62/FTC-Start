package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.logging.XMLFormatter;

@Autonomous
public class AutoRedGrabber extends LinearOpMode {
    double X = .15;
    double Y = 0;
    int stage =  0;
    @Override
    public void runOpMode() throws InterruptedException {

        //Registering Dc Motors
        DcMotor BackLeft; BackLeft = hardwareMap.get(DcMotor.class,   "bLeft");
        DcMotor FrontLeft; FrontLeft = hardwareMap.get(DcMotor.class,   "fLeft");
        DcMotor BackRight; BackRight = hardwareMap.get(DcMotor.class,   "bRight");
        DcMotor FrontRight; FrontRight = hardwareMap.get(DcMotor.class,   "fRight");

        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Registering Sensors
        DistanceSensor Distance;
        ColorSensor Color;
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");
        Color = hardwareMap .get(ColorSensor.class, "Color");

        //Registering Servos
        Servo base;
        Servo wrist;
        Servo claw;
//hi ryan
        base = hardwareMap.get(Servo.class,  "arm");
        wrist = hardwareMap.get(Servo.class,  "wrist");
        claw = hardwareMap.get(Servo.class,  "claw");

        base.scaleRange(0, 0.54);
        wrist.scaleRange(0, 0.27);
        claw.scaleRange(0, 0.4);

        base.setPosition(1); //INIT positions
        wrist.setPosition(1);
        claw.setPosition(0);

        IMU imu; imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                (new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT))));
        double correction = 0;

        double startangle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        waitForStart();
        double Red2 = Color.red();

        while(opModeIsActive()){
            //Sensors
            double Red = Color.red();
            double Green = Color.green();
            double Blue =  Color.blue();
            double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            //Drive
            if (Math.abs(angle - startangle) > 0.5){
                correction = -(angle - startangle) * 0.01;
                correction = Range.clip(correction, -0.1, 0.1);
            };
            double turn = correction;
            double strafe = X;
            double speed = Y;

            if (Red > Red2 + 1 ){
                claw.setPosition(1);
                X = 0;
                
                sleep(1000);
                base.setPosition(0);
                wrist.setPosition(0);
                sleep(500);
                claw.setPosition(0);
                Red2 = 10000;
            }


            FrontLeft.setPower(speed + turn - strafe);
            FrontRight.setPower(-speed + turn - strafe);
            BackLeft.setPower(speed + turn + strafe);
            BackRight.setPower(-speed + turn + strafe);

            //Telementary
            telemetry.addData("Y:", Y);
            telemetry.addData("X:", X);
            telemetry.addData("Angle:", angle);
            telemetry.addData("Red", Red);
            telemetry.addData("Green", Green);
            telemetry.addData("Blue", Blue);
            telemetry.update();


            //METHODS


        }

    }


}
