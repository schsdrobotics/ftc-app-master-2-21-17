/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
    Program first created by Steven Velez-Garcia on 10/07/2015
    Program template finalized by Steven Velez-Garcia on 02/15/2016
    Program modifided by Johnathan, Otto, and Thomas on 9/15/2016
    Program Further Modified by Thomas and Gabe on 12/29/16
 */

@TeleOp(name="Teleop 2-8-17", group="2017")  // @Autonomous(...) is the other common choice
public class Teleop_code_2017 extends LinearOpMode {


  //  double servoIncrement = .03;



    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    Harderware robot = new Harderware();



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
//        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
//        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
//        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
//        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
//        //sweeperMotor = hardwareMap.dcMotor.get("sweeperMotor");
//        //liftMotor = hardwareMap.dcMotor.get("liftMotor");

//        // eg: Set the drive motor directions:
//        // "Reverse" the motor that runs backwards when connected directly to the battery
//        rightBackMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//        //sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE); //May be removed if motor is in without need

        robot.holdMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        int controlToggle = -1;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

//          robot.releaseServo.setPosition(0);
//
            /////////////Joystick Movement/////////////

            //Takes values of joysticks for movement of robot

            //Clips motor values so it doesn't try to give it more power than possible

            //strafe =  Range.clip(strafe, -1, 1);

            if(gamepad1.start){
                controlToggle = -1*controlToggle;
                sleep(250);
            }

            //Takes previous values and sets to motor value (already scaled)
            if(controlToggle == 1) {
                float speed = -gamepad1.left_stick_y;
                float direction = gamepad1.left_stick_x;
                float right = speed - direction;
                float left = speed + direction;

                right =  Range.clip(right, -1, 1);
                left =  Range.clip(left, -1, 1);

                robot.leftFrontMotor.setPower(left);
                robot.leftBackMotor.setPower(left);
                robot.rightFrontMotor.setPower(right);
                robot.rightBackMotor.setPower(right);
            }
            if(controlToggle == -1) {
                float speed = -gamepad1.left_stick_y;
                float direction = gamepad1.left_stick_x;
                float right = speed + direction;
                float left = speed - direction;

                right =  Range.clip(right, -1, 1);
                left =  Range.clip(left, -1, 1);

                robot.leftFrontMotor.setPower(controlToggle * left);
                robot.leftBackMotor.setPower(controlToggle * left);
                robot.rightFrontMotor.setPower(controlToggle * right);
                robot.rightBackMotor.setPower(controlToggle * right);
            }



            /*
            If the thought process is correct, not moving the joysticks and pressing the triggers
            will allow us to strafe. All have the same value, except they are negated in order to
            use the correct convention of wheel directions when driving. Convention can be changed
            if inverted, this is only a theoretical section until we figure out a way to strafe and
            move all at the same time.
            Edited at 4:41, 10/5/2016

            We work now.

            Edited at 6:32, 10/7/2016
            */

  /*          if (left == 0 & right == 0) {
                robot.leftFrontMotor.setPower(-strafe);
                robot.leftBackMotor.setPower(strafe);
                robot.rightBackMotor.setPower(-strafe);
                robot.rightFrontMotor.setPower(strafe);
                }
*/
//
            /////////////Single Button Controls/////////////

            if (gamepad2.right_bumper){
                robot.catapultMotor.setPower(1);
            } else {
                robot.catapultMotor.setPower(0);
            }


            if (gamepad2.dpad_up){
                robot.liftMotor.setPower(1);
            } else if (gamepad2.dpad_down){
                robot.liftMotor.setPower(-1);
            } else {
                robot.liftMotor.setPower(0);
            }

            if (gamepad2.a){
                robot.holdMotor.setPower(1.0);
            } else if (gamepad2.b) {
                robot.holdMotor.setPower(-1.0);
            } else {
                robot.holdMotor.setPower(0);
            }

            if (gamepad2.left_bumper) {
                robot.basketServo.setPosition(1);
            }
/*



            if (gamepad2.dpad_left){
                robot.buttonServo.setPosition(robot.buttonServo.getPosition() - servoIncrement);
            }

            if (gamepad2.dpad_right){
                robot.buttonServo.setPosition(robot.buttonServo.getPosition() + servoIncrement);
            }
*/
            telemetry.addData("Control Direction", controlToggle);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}