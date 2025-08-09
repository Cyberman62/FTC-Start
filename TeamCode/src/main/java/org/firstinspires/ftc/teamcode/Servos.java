package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Servos extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
    //Init Code  Goes Here

        Servo base;   //servoOne
        Servo wrist;   //servoTwo
        Servo claw;   //servoThree

        base = hardwareMap.get(Servo.class,  "arm");
        wrist = hardwareMap.get(Servo.class,  "wrist");
        claw = hardwareMap.get(Servo.class,  "claw");


        base.scaleRange(0, 0.54);
        wrist.scaleRange(0, 0.23);
        claw.scaleRange(0, 0.4);

        base.setPosition(1);
        wrist.setPosition(1);
        claw.setPosition(1);

        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.right_trigger == 0){
                base.setPosition(base.getPosition() - (gamepad1.left_trigger/300));
                wrist.setPosition(wrist.getPosition() - (gamepad1.left_trigger/300));
            }

            if (gamepad1.left_trigger == 0){
                base.setPosition(base.getPosition() + (gamepad1.right_trigger/300));
                wrist.setPosition(wrist.getPosition() + (gamepad1.right_trigger/300));
            }

            if (gamepad1.right_bumper) {
                claw.setPosition(0);
            } else if (gamepad1.left_bumper){
                claw.setPosition(1);
            }

            telemetry.addData("Claw Position", claw.getPosition()) ;
            telemetry.update();


        }
    }
}
