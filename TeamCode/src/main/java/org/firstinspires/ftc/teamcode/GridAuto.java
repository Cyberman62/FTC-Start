package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.logging.XMLFormatter;

@Autonomous
public class GridAuto extends LinearOpMode {
    double Dist;
    double Time = 0;
    ElapsedTime timer = new ElapsedTime();
    double X = -0.25;
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
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");

        //Registering Servos
        Servo base;
        Servo wrist;
        Servo claw;
//hi ryan
        base = hardwareMap.get(Servo.class,  "arm");
        wrist = hardwareMap.get(Servo.class,  "wrist");
        claw = hardwareMap.get(Servo.class,  "claw");

        base.scaleRange(0, 0.54);
        wrist.scaleRange(0, 0.23);
        claw.scaleRange(0, 0.4);

        base.setPosition(0); //INIT positions
        wrist.setPosition(0);
        claw.setPosition(1);

        IMU imu; imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                (new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT))));
     double correction = 0;

        double startangle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        waitForStart();
        sleep(100);
        timer.reset();

        while(opModeIsActive()){
            //Sensors
            Dist = Distance.getDistance(DistanceUnit.INCH);
            double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            //Drive
            if (Math.abs(angle - startangle) > 0.5){
                correction = -(angle - startangle) * 0.01;
                correction = Range.clip(correction, -0.1, 0.1);
            };
            double turn = correction;
            double strafe = X;
            double speed = Y;

                sweep();


            FrontLeft.setPower(speed + turn - strafe);
            FrontRight.setPower(-speed + turn - strafe);
            BackLeft.setPower(speed + turn + strafe);
            BackRight.setPower(-speed + turn + strafe);

            //Telementary
            telemetry.addData("Distance: ", Dist);
            telemetry.addData("Y:", Y);
            telemetry.addData("X:", X);
            telemetry.addData("Angle:", angle);
            telemetry.addData("Time:", Time);
            telemetry.addData("Timer:", timer.seconds());
            telemetry.update();


            //METHODS


        }
    }        void sweep() {
        if (Dist < 10 && Dist > 2 && stage == 0){
            if (Time == 0){
                Time = getRuntime();
            }
            timer.reset();
            Y = 0.15;
            X = 0;
            stage = 1;
        }
        if(timer.seconds() >= 1.25 && stage == 1){
            timer.reset();
            Y = 0;
            X = .25;
            stage = 2;
        }
        if(timer.seconds() >= (Time - 2.12) && stage == 2){
            timer.reset();
            X = 0;
            Y = .15;
            stage = 3;
        }
        if(timer.seconds() >= 1.25 && stage == 3){
            Y = 0;
            X = -0.25;
            timer.reset();
            stage = 0;
        }

    }


}
