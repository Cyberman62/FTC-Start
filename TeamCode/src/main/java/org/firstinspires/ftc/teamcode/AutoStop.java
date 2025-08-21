//IMPORTANT: MUST HAVE DISTANCE SENSOR ON BACK
package org.firstinspires.ftc.teamcode;


import android.graphics.Color;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class AutoStop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();


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
        Distance = hardwareMap.get(DistanceSensor.class, "Distance2");


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





    int Loops = 0;


        waitForStart();
        boolean party = false;
        int step = 0;


        while(opModeIsActive()){

            if  (party == false) {
                //Sensors
                double Dist = Distance.getDistance(DistanceUnit.INCH);


                //Drive
                double turn = -gamepad1.right_stick_x * 0.3;
                double strafe = gamepad1.left_stick_x * .5;
                double speed = -gamepad1.left_stick_y * .5;


                if (gamepad1.left_stick_button) {
                    speed = -gamepad1.left_stick_y;
                    strafe = gamepad1.left_stick_x;
                } else if (gamepad1.right_stick_button) {
                    turn = -gamepad1.right_stick_x * 0.6;
                }


                double speed2 = Math.abs(speed) + 1;
                double speed3 = Math.pow(speed2, 3.5);
                double backup = (3 + Math.abs(speed3));


                if (Dist < backup && speed < 0) {
                    speed = 0;
                    gamepad1.rumble(100);
                }


                FrontLeft.setPower(speed + turn - strafe);
                FrontRight.setPower(-speed + turn - strafe);
                BackLeft.setPower(speed + turn + strafe);
                BackRight.setPower(-speed + turn + strafe);


                //Arm
                if (gamepad1.right_trigger == 0 && gamepad1.left_trigger > 0.2) {
                    base.setPosition(0);
                    wrist.setPosition(0);
                }

                if (gamepad1.left_trigger == 0 && gamepad1.right_trigger > 0.2) {
                    base.setPosition(1);
                    wrist.setPosition(1);
                }


                //Claw
                if (gamepad1.right_bumper) {
                    claw.setPosition(0);
                } else if (gamepad1.left_bumper) {
                    claw.setPosition(1);
                }


                Loops = Loops + 1;

                //Telementary
                telemetry.addData("Distance: ", Dist);
                telemetry.addData("Distance Until Stop: ", backup);
                telemetry.addData("step", step);
                telemetry.addData("Loops", (Loops));
                telemetry.update();

            }




            if (timer.milliseconds() > 1500) {
                step = 0;
            }


            // Step 0: waiting for first UP
            if (step == 0 && gamepad1.dpad_up) {
                step = 1;
                timer.reset();
            }
            // Step 1: waiting for second UP
            else if (step == 1 && gamepad1.dpad_up) {
                step = 2;
                timer.reset();
            }
            // Step 2: waiting for first DOWN
            else if (step == 2 && gamepad1.dpad_down) {
                step = 3;
                timer.reset();
            }
            // Step 3: waiting for second DOWN
            else if (step == 3 && gamepad1.dpad_down) {
                step = 4;
                timer.reset();
            }
            // Step 4: waiting for RIGHT
            else if (step == 4 && gamepad1.dpad_right) {
                step = 5;
                timer.reset();
            }
            // Step 5: waiting for LEFT
            else if (step == 5 && gamepad1.dpad_left) {
                step = 6;
                timer.reset();
            }
            // Step 6: waiting for RIGHT
            else if (step == 6 && gamepad1.dpad_right) {
                step = 7;
                timer.reset();
            }
            // Step 7: waiting for second RIGHT
            else if (step == 7 && gamepad1.dpad_right) {
                party = true;
            }


            if (party) {
                base.setPosition(0.7);
                if (claw.getPosition() == 0) {
                    claw.setPosition(1);
                }

                if (claw.getPosition() == 1) {
                    claw.setPosition(0);
                }


                long now = System.currentTimeMillis();


                double hue = (now % 5000) / 5000.0 * 360.0;  // 5s full cycle


                float[] hsv = {(float) hue, 1f, 1f};


                int rgb = Color.HSVToColor(hsv);


                double r = ((rgb >> 16) & 0xFF) / 255.0;
                double g = ((rgb >> 8) & 0xFF) / 255.0;
                double b = (rgb & 0xFF) / 255.0;


                gamepad1.setLedColor(r, g, b, 50);


                telemetry.addLine("**Party Mode Activated**");
                telemetry.addLine("*will continue until  stop button pressed");
                telemetry.update();
            }




        }
    }
}


