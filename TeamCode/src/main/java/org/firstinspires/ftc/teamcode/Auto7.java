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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Auto7", group = "2017")

public class Auto7 extends LinearOpMode {

    /* Declare OpMode members. */
    Harderware robot = new Harderware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final int DRIVE_LEFT = 0;
    static final int DRIVE_RIGHT = 1;
    static final int REVERSE = -1;
    static final int FORWARD = 1;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "L(%7d) | R(%7d)", robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        sleep(15000);
        encoderDriveStraight(DRIVE_SPEED, 84, 5.0);

        /**
         * ################################## MOVEMENT END #########################################
         */

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void startMotion(double speed){
        // Turn On RUN_TO_POSITION
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.leftFrontMotor.setPower(Math.abs(speed));
        robot.leftBackMotor.setPower(Math.abs(speed));
        robot.rightFrontMotor.setPower(Math.abs(speed));
        robot.rightBackMotor.setPower(Math.abs(speed));
    }

    public void endMotion(){
        // Stop all motion;
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightBackMotor.setPower(0);

        // Turn off RUN_TO_POSITION mode
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void rotateTime(double speed, double time, int direction) {

        double currentRuntime = runtime.seconds();

        if (direction == 0){
            while (runtime.seconds() <= currentRuntime + time) {
                robot.leftFrontMotor.setPower(-speed);
                robot.leftBackMotor.setPower(-speed);
                robot.rightFrontMotor.setPower(speed);
                robot.rightBackMotor.setPower(speed);
            }
        } else if (direction == 1){
            while (runtime.seconds() <= currentRuntime + time) {
                robot.leftFrontMotor.setPower(speed);
                robot.leftBackMotor.setPower(speed);
                robot.rightFrontMotor.setPower(-speed);
                robot.rightBackMotor.setPower(-speed);
            }
        }


    }

    /*
     *  FORWARD Drive POSITIVE distance / REVERSE Drive NEGATIVE dsitance
     */
    public void encoderDriveStraight(double speed, double distance, double timeoutS) {

        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // change mode and start motion
            startMotion(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Run to L(%7d) | R(%7d)", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at L(%7d) | R(%7d)", robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop motion and reset motor mode
            endMotion();
        }
    }

    /*
     *  Strafe Direction: 0 = left, 1 = right
     */

    public void encoderDriveStrafe(int direction, double speed, double distance, double timeoutS) {

        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            if (direction == 0){
                newLeftFrontTarget *= REVERSE;
                newLeftBackTarget *= FORWARD;
                newRightFrontTarget *= FORWARD;
                newRightBackTarget *= REVERSE;
            }
            else if (direction == 1) {
                newLeftFrontTarget *= FORWARD;
                newLeftBackTarget *= REVERSE;
                newRightFrontTarget *= REVERSE;
                newRightBackTarget *= FORWARD;
            }

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // change mode and start motion
            startMotion(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Run to L(%7d) | R(%7d)", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at L(%7d) | R(%7d)", robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop motion and reset motor mode
            endMotion();
        }
    }
}
