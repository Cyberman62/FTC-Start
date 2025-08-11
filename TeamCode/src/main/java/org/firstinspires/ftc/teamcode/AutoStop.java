//IMPORTANT: MUST HAVE DISTANCE SENSOR ON BACK
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class AutoStop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //Registering Dc Motors
        DcMotor BackLeft;
        BackLeft = hardwareMap.get(DcMotor.class,   "bLeft");
        DcMotor FrontLeft;
        FrontLeft = hardwareMap.get(DcMotor.class,   "fLeft");
        DcMotor BackRight;
        BackRight = hardwareMap.get(DcMotor.class,   "bRight");
        DcMotor FrontRight;
        FrontRight = hardwareMap.get(DcMotor.class,   "fRight");

        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        base.setPosition(1); //INIT positions
        wrist.setPosition(1);
        claw.setPosition(1);


        waitForStart();

        while(opModeIsActive()){
            //Sensors
            double Dist = Distance.getDistance(DistanceUnit.INCH);

            //Drive
            double turn = -gamepad1.right_stick_x * 0.3;
            double strafe = gamepad1.left_stick_x * .5;
            double speed = -gamepad1.left_stick_y * .5;

            if(gamepad1.left_stick_button){
                speed = -gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x;
            }
            else if(gamepad1.right_stick_button){
                turn = -gamepad1.right_stick_x * 0.6;
            }

            double speed2 = Math.abs(speed) + 1;
            double speed3 = Math.pow(speed2, 3.5);
            double backup = (3 + Math.abs(speed3));

            if(Dist < backup && speed < 0){
                speed = 0;
                gamepad1.rumble(100);
            }

            FrontLeft.setPower(speed + turn - strafe);
            FrontRight.setPower(-speed + turn - strafe);
            BackLeft.setPower(speed + turn + strafe);
            BackRight.setPower(-speed + turn + strafe);

            //Arm
            if (gamepad1.right_trigger == 0){
                base.setPosition(base.getPosition() - (gamepad1.left_trigger/30));
                wrist.setPosition(wrist.getPosition() - (gamepad1.left_trigger/30));
            }

            if (gamepad1.left_trigger == 0){
                base.setPosition(base.getPosition() + (gamepad1.right_trigger/30));
                wrist.setPosition(wrist.getPosition() + (gamepad1.right_trigger/30));
            }

            //Claw
            if (gamepad1.right_bumper) {
                claw.setPosition(0);
            } else if (gamepad1.left_bumper){
                claw.setPosition(1);
            }


            //Telementary
            telemetry.addData("Distance: ", Dist);
            telemetry.addData("Distance Until Stop: ", backup);
            telemetry.addLine ("Speeds:");
            telemetry.addData("Speed1:", speed);
            telemetry.addData("Speed2:", speed2);
            telemetry.addData("Speed3:", speed3);
            telemetry.update();


        }
    }
}

