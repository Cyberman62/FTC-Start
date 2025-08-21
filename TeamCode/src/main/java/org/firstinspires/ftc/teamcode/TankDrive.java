
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//Letting the code know some important  stuff - Android Studio will automatically import as you need

@TeleOp //This lets the bot know that this program  should show up under the TeleOp section, instead of Autonomous

public class TankDrive extends LinearOpMode { //Make sure the name  here  matches up  with  the name of this project. In this case, its "TankDrive".

    @Override

    public void runOpMode() throws InterruptedException {
        //The code here will run when you press "INIT" on the  driver station.

        //Telling the robot that there are 2 DcMotors, one named Right, the other named  Left.
        DcMotor Right;
        DcMotor Left;

        //Telling the robot what the motors are called  in the  configuration  on the drivers station.
        Right = hardwareMap.get(DcMotor.class, "RightMotor");
        Left = hardwareMap.get(DcMotor.class, "LeftMotor");

        /*
        First OPTIONAL piece of code. This makes the motors stop rotating when no power is being assigned  to them.

         There are 2 options:

        FLOAT - Default, will make motors coast down to a stop.
        BRAKE - Will immediately stop, and make the motors hard to turn, until power is given.
        */
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Sets the  default direction for the motor's rotation.
        //The directions are reversed because when the motors are on either side, they spin in opposite directions.
        Right.setDirection(DcMotorSimple.Direction.REVERSE);
        Left.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        //Code here  runs one time after start is pressed on the  driver station.

        while(opModeIsActive()) {
            //code here will loop until stop is pressed on the driver hub.

            //Code  for  tank  drive - it sets power to the motors with Right/Left.setPower()
            //It sets the power to the joystick's Y. One thing to note about joysticks is that for  the Y axis, Down on the  Y gives a positive value, so it is inverted.
            //Joysticks max  out at a power of 1 for all the  way to the edge. Motors also  have it so 1 is max power.

            Right.setPower(gamepad1.right_stick_y * -1);
            //Due  to the Y axis being inverted, we multiply it by -1 to flip it.

            Left.setPower(-gamepad1.left_stick_y);
            //to get negative values, we can also just  add  a - before it.
        }

        }
    }

